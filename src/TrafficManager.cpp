#include <stdio.h>
#include <string.h>
#include "../include/djik.h"
#include <stdlib.h>
#include <fstream>
#include <list>
#include "../include/TrafficManager.h"
#include <pthread.h>
#include <signal.h>
#include <vector>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>

//dia of wheel new AGV 125mm
//TODO:
//Speed of AGV between RFIDs
//Odometry to be made correct
//For Space_time_djik and path to time_plan
#if LIVE_MODE == 0
#define SAME_SPEED_SIMULATION 1
#else
#define SAME_SPEED_SIMULATION 0
#endif

#define AUTO_CHARGING 0

#if FULL_DEMO
#define PICKUP_NODE_NUM 7
#define REPLENISH_NODE_NUM 2
#elif EXPO_DEMO
#define PICKUP_NODE_NUM 27
#define REPLENISH_NODE_NUM 27

#elif VAHAK_FULL_DEMO
#define PICKUP_NODE_NUM 20
#define REPLENISH_NODE_NUM 20
#elif SHOPFLOOR_DEMO_SMALL
#define PICKUP_NODE_NUM 1
#define REPLENISH_NODE_NUM 1
#elif SHOPFLOOR_INTERNAL
#define PICKUP_NODE_NUM 21
#define REPLENISH_NODE_NUM 18
#elif WAREHOUSE_DEMO
#define PICKUP_NODE_NUM 1
#define REPLENISH_NODE_NUM 1
#endif

#define STATIC_WAIT_TIME 5
#define PAUSE_TIME_STEPS 3 //Pause time steps to add in time plan Data-structure

#define PICKUP_REPLENISH_TIME_WAIT 5
#define BEHAVIOR_CODING 1
#define EDGE_COST_SIMULATION 10 // Edge cost in simulation

#define DIVERTER_LOGIC 0 //set 1 to enable Diverter checking code - use only when Rackmanager is working
#define LIFTER_LOGIC 0 //set 1 to enable Lifter checking code - use only when Lift manager is working

#if LIVE_MODE == 1
#define TIME_WINDOW 9 //how far ahead to check for collisions
#define ERASE_PREV_PLAN 1 //to deal with different AGV speeds
#else
#define TIME_WINDOW 20 //how far ahead to check for collisions
#define ERASE_PREV_PLAN 1
#endif

//#define MAIN_USE

using namespace std;
pthread_t timeplan_update_thread, simulate_thread;

enum agent_behaviour {left_fork = 1, right_fork, center_tape, rack, pick_bin, pause_agv, put_bin, pick_station, replenish_station, center_tape1, rack1, rack2};
static std::map<agent_behaviour, QString> map_behave_enumtostring;
static std::map<QString, int> map_behave_stringtoint;

void Initialize_Behavior_enumtostring(void)
{
    map_behave_enumtostring[left_fork] = "left"; //1
    map_behave_enumtostring[right_fork] = "right"; //2
    map_behave_enumtostring[pause_agv] = "pause"; //6
    map_behave_enumtostring[center_tape] = "center"; //3
    map_behave_enumtostring[rack] = "rack"; //4
    map_behave_enumtostring[pick_bin] = "pick_bin"; //5
    map_behave_enumtostring[put_bin] = "put_bin"; //7
    map_behave_enumtostring[pick_station] = "pick_station"; //8
    map_behave_enumtostring[replenish_station] = "replenish_station"; //9
    map_behave_enumtostring[center_tape1] = "center_tape1"; //10
    map_behave_enumtostring[rack1] = "rack1"; //11
    map_behave_enumtostring[rack2] = "rack2"; //12
}

void Initialize_Behaviour_stringtoint(void)
{
    map_behave_stringtoint["left"] = (int)left_fork;
    map_behave_stringtoint["right"] = (int)right_fork;
    map_behave_stringtoint["pause"] = (int)pause_agv;
    map_behave_stringtoint["center"] = (int)center_tape;
    map_behave_stringtoint["rack"] = (int)rack;
    map_behave_stringtoint["pick_bin"] = (int)pick_bin;
    map_behave_stringtoint["put_bin"] = (int)put_bin;
    map_behave_stringtoint["pick_station"] = (int)pick_station;
    map_behave_stringtoint["replenish_station"] = (int)replenish_station;
    map_behave_stringtoint["center_tape1"] = (int)center_tape1;
    map_behave_stringtoint["rack1"] = (int)rack1;
    map_behave_stringtoint["rack2"] = (int)rack2;
}

/*
   Maintaining local RFID to Node and vice versa mapping
   TBD: move this to database
*/

// Value-Defintions of the different String values
enum Nodenum { n0=0,n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,\
               n18 = 18, n19, n20, n21, n22, n23,n24, n25, n26 = 26, n27, n28, n29, n30, n31, n32, n33, n34, n35, n36 };
// Map to associate the strings with the enum values
static std::map<QString, Nodenum> s_mapRFIDtoNode;
static std::map<Nodenum, QString> s_mapNodetoRFID;

enum diverter_request {NOT_REQUESTED_DIVERTER, DIVERTER_REQUEST_PENDING, DIVERTER_REQUEST_ACCEPTED};

void Initialize_NodetoRFID_map(void)
{
#if EXPO_DEMO
    s_mapNodetoRFID[n27] = "PPPPCCC001";
    s_mapNodetoRFID[n26] = "FFFFAAA003";
    s_mapNodetoRFID[n25] = "FFFFAAA002";
    s_mapNodetoRFID[n24] = "FFFFAAA001";
    s_mapNodetoRFID[n23] = "LLLLAAA001";
    s_mapNodetoRFID[n22] = "9999AAA001";
    s_mapNodetoRFID[n21] = "9999AAA002";
    s_mapNodetoRFID[n20] = "9999AAA003";
    s_mapNodetoRFID[n19] = "9999AAA004";
    s_mapNodetoRFID[n18] = "9999AAA005";
    s_mapNodetoRFID[n17] = "LLLLBBB001";
    s_mapNodetoRFID[n16] = "9999BBB001";
    s_mapNodetoRFID[n15] = "9999BBB002";
    s_mapNodetoRFID[n14] = "9999BBB003";
    s_mapNodetoRFID[n13] = "9999BBB004";
    s_mapNodetoRFID[n12] = "9999BBB005";
    s_mapNodetoRFID[n11] = "LLLLCCC001";
    s_mapNodetoRFID[n10] = "9999CCC001";
    s_mapNodetoRFID[n9] = "9999CCC002";
    s_mapNodetoRFID[n8] = "9999CCC003";
    s_mapNodetoRFID[n7] = "9999CCC004";
    s_mapNodetoRFID[n6] = "9999CCC005";
    s_mapNodetoRFID[n5] = "LLLLDDD001";
    s_mapNodetoRFID[n4] = "9999DDD001";
    s_mapNodetoRFID[n3] = "9999DDD002";
    s_mapNodetoRFID[n2] = "9999DDD003";
    s_mapNodetoRFID[n1] = "9999DDD004";
    s_mapNodetoRFID[n0] = "9999DDD005";

#elif FULL_DEMO
    s_mapNodetoRFID[n0] = "01002FFEF0"; s_mapNodetoRFID[n1] = "0100229B47";
    s_mapNodetoRFID[n2] = "0100248599"; s_mapNodetoRFID[n3] = "01002F27BB";
    s_mapNodetoRFID[n4] = "010023B4DD"; s_mapNodetoRFID[n5] = "010027C273";
    s_mapNodetoRFID[n6] = "01002F51D2"; s_mapNodetoRFID[n7] = "01002BE8AB";
    s_mapNodetoRFID[n8] = "010028A404"; s_mapNodetoRFID[n9] = "01002BD6D8";
    s_mapNodetoRFID[n10] = "01002C28BA"; s_mapNodetoRFID[n11] = "01002A89C5";
    s_mapNodetoRFID[n12] = "0100250CD9"; s_mapNodetoRFID[n13] = "01002CDED4";
    s_mapNodetoRFID[n14] = "0100285093"; s_mapNodetoRFID[n15] = "010022B278";
    s_mapNodetoRFID[n16] = "0100279BC4"; s_mapNodetoRFID[n17] = "01002646EA";
    s_mapNodetoRFID[n18] = "0100238BF3"; s_mapNodetoRFID[n19] = "010023693A";
    s_mapNodetoRFID[n20] = "0100207C40"; s_mapNodetoRFID[n21] = "010025562C";
    s_mapNodetoRFID[n22] = "0100231B3B"; s_mapNodetoRFID[n23] = "01002F1975";
    s_mapNodetoRFID[n24] = "0100294765"; s_mapNodetoRFID[n25] = "01002E5F74";
    s_mapNodetoRFID[n26] = "01002AA625"; s_mapNodetoRFID[n27] = "010027FA4E";
    s_mapNodetoRFID[n28] = "010028F854"; s_mapNodetoRFID[n29] = "010022EAA3";
    s_mapNodetoRFID[n30] = "010024BE91"; s_mapNodetoRFID[n31] = "010021E0E8";
    s_mapNodetoRFID[n32] = "01002B24BC"; s_mapNodetoRFID[n33] = "01002B6843";
    s_mapNodetoRFID[n34] = "0100249406"; s_mapNodetoRFID[n35] = "010026DB03";
    s_mapNodetoRFID[n36] = "0100212345";
#elif VAHAK_FULL_DEMO
    s_mapNodetoRFID[n20] = "01002BED61";
    s_mapNodetoRFID[n19] = "01002BF3D0";
    s_mapNodetoRFID[n18] = "02002E5D4F";
    s_mapNodetoRFID[n17] = "9999AAA003";
    s_mapNodetoRFID[n16] = "9999AAA002";
    s_mapNodetoRFID[n15] = "9999AAA001";
    //s_mapNodetoRFID[n17] = "9999AAA006";
    //s_mapNodetoRFID[n16] = "9999AAA005";
    //s_mapNodetoRFID[n15] = "9999AAA004";
    s_mapNodetoRFID[n14] = "02002E54D2";
    s_mapNodetoRFID[n13] = "02002E46FF";
    s_mapNodetoRFID[n12] = "02002E5B32";
    s_mapNodetoRFID[n11] = "LLLLBBB002";
    s_mapNodetoRFID[n10] = "9999BBB003";
    s_mapNodetoRFID[n9] = "9999BBB002";
    s_mapNodetoRFID[n8] = "9999BBB001";
    //s_mapNodetoRFID[n10] = "9999BBB006";
    //s_mapNodetoRFID[n9] = "9999BBB005";
    //s_mapNodetoRFID[n8] = "9999BBB004";
    s_mapNodetoRFID[n7] = "LLLLBBB001";
    s_mapNodetoRFID[n6] = "02002E5117";
    s_mapNodetoRFID[n5] = "02002E4DEC";
    s_mapNodetoRFID[n4] = "02002E6358";
    s_mapNodetoRFID[n3] = "01002B48AA";
    s_mapNodetoRFID[n2] = "02002E5028";
    s_mapNodetoRFID[n1] = "02002E54A5";
    s_mapNodetoRFID[n0] = "02002E4AB4";
#elif SHOPFLOOR_DEMO_SMALL
    s_mapNodetoRFID[n0] = "01002BF3D0"; s_mapNodetoRFID[n1] = "01002BED61";
    s_mapNodetoRFID[n2] = "01002B8059"; s_mapNodetoRFID[n3] = "0100272B26";
    s_mapNodetoRFID[n4] = "01002B8BD3"; s_mapNodetoRFID[n5] = "0100275FE4";
    s_mapNodetoRFID[n6] = "0300FF794D"; s_mapNodetoRFID[n7] = "01002B76CC";
    s_mapNodetoRFID[n8] = "55004EC9F9"; s_mapNodetoRFID[n9] = "03009AB310";
    s_mapNodetoRFID[n10] = "01002B48AA"; s_mapNodetoRFID[n11] = "01002B8BCE";
    s_mapNodetoRFID[n12] = "01002BE926"; s_mapNodetoRFID[n13] = "01002BD622";
    s_mapNodetoRFID[n14] = "01002BBD98";
#elif WAREHOUSE_DEMO
    s_mapNodetoRFID[n0] = "R1"; s_mapNodetoRFID[n1] = "R2";
    s_mapNodetoRFID[n2] = "R3"; s_mapNodetoRFID[n3] = "R4";
    s_mapNodetoRFID[n4] = "R5"; s_mapNodetoRFID[n5] = "R6";
#elif SHOPFLOOR_INTERNAL
    s_mapNodetoRFID[n22] = "01002BF3D0";
    s_mapNodetoRFID[n21] = "01002BED61";
    s_mapNodetoRFID[n20] = "01002B8059";
    s_mapNodetoRFID[n19] = "0100272B26";
    s_mapNodetoRFID[n18] = "01002B8BD3";
    s_mapNodetoRFID[n17] = "0100275FE4";
    s_mapNodetoRFID[n16] = "01002BBD98";
    s_mapNodetoRFID[n15] = "0300FF794D";
    s_mapNodetoRFID[n14] = "03009B720A";
    s_mapNodetoRFID[n13] = "01002BD622";
    s_mapNodetoRFID[n12] = "01002B48AA";
    s_mapNodetoRFID[n11] = "01002B76CC";
    s_mapNodetoRFID[n10] = "01002BA950";
    s_mapNodetoRFID[n9] = "02009B48AC";
    s_mapNodetoRFID[n8] = "010026F95B";
    s_mapNodetoRFID[n7] = "01002BEB4C";
    s_mapNodetoRFID[n6] = "0300FF5996";
    s_mapNodetoRFID[n5] = "0300FF5AF2";
    s_mapNodetoRFID[n4] = "9999AAA003";
    s_mapNodetoRFID[n3] = "9999AAA002";
    s_mapNodetoRFID[n2] = "9999AAA001";
    s_mapNodetoRFID[n1] = "55004F2ACA";
    s_mapNodetoRFID[n0] = "03009B614A";
#endif
}

void Initialize_RFIDtoNode_map(void)
{
#if EXPO_DEMO
    s_mapRFIDtoNode["PPPPCCC001"] = n27;
    s_mapRFIDtoNode["PPPPCCC009"] = n27;
    s_mapRFIDtoNode["FFFFAAA003"] = n26;
    s_mapRFIDtoNode["FFFFAAA002"] = n25;
    s_mapRFIDtoNode["FFFFAAA001"] = n24;
    s_mapRFIDtoNode["LLLLAAA001"] = n23;
    s_mapRFIDtoNode["9999AAA001"] = n22;
    s_mapRFIDtoNode["9999AAA002"] = n21;
    s_mapRFIDtoNode["9999AAA003"] = n20;
    s_mapRFIDtoNode["9999AAA004"] = n19;
    s_mapRFIDtoNode["9999AAA005"] = n18;

    s_mapRFIDtoNode["9999AAA006"] = n22;
    s_mapRFIDtoNode["9999AAA007"] = n21;
    s_mapRFIDtoNode["9999AAA008"] = n20;
    s_mapRFIDtoNode["9999AAA009"] = n19;
    s_mapRFIDtoNode["9999AAA010"] = n18;

    s_mapRFIDtoNode["LLLLBBB001"] = n17;
    s_mapRFIDtoNode["9999BBB001"] = n16;
    s_mapRFIDtoNode["9999BBB002"] = n15;
    s_mapRFIDtoNode["9999BBB003"] = n14;
    s_mapRFIDtoNode["9999BBB004"] = n13;
    s_mapRFIDtoNode["9999BBB005"] = n12;

    s_mapRFIDtoNode["9999BBB006"] = n16;
    s_mapRFIDtoNode["9999BBB007"] = n15;
    s_mapRFIDtoNode["9999BBB008"] = n14;
    s_mapRFIDtoNode["9999BBB009"] = n13;
    s_mapRFIDtoNode["9999BBB010"] = n12;

    s_mapRFIDtoNode["LLLLCCC001"] = n11;
    s_mapRFIDtoNode["9999CCC001"] = n10;
    s_mapRFIDtoNode["9999CCC002"] = n9;
    s_mapRFIDtoNode["9999CCC003"] = n8;
    s_mapRFIDtoNode["9999CCC004"] = n7;
    s_mapRFIDtoNode["9999CCC005"] = n6;

    s_mapRFIDtoNode["9999CCC006"] = n10;
    s_mapRFIDtoNode["9999CCC007"] = n9;
    s_mapRFIDtoNode["9999CCC008"] = n8;
    s_mapRFIDtoNode["9999CCC009"] = n7;
    s_mapRFIDtoNode["9999CCC010"] = n6;

    s_mapRFIDtoNode["LLLLDDD001"] = n5;
    s_mapRFIDtoNode["9999DDD001"] = n4;
    s_mapRFIDtoNode["9999DDD002"] = n3;
    s_mapRFIDtoNode["9999DDD003"] = n2;
    s_mapRFIDtoNode["9999DDD004"] = n1;
    s_mapRFIDtoNode["9999DDD005"] = n0;

    s_mapRFIDtoNode["9999DDD006"] = n4;
    s_mapRFIDtoNode["9999DDD007"] = n3;
    s_mapRFIDtoNode["9999DDD008"] = n2;
    s_mapRFIDtoNode["9999DDD009"] = n1;
    s_mapRFIDtoNode["9999DDD010"] = n0;

#elif FULL_DEMO
    s_mapRFIDtoNode["01002FFEF0"] = n0; s_mapRFIDtoNode["0100229B47"] = n1;
    s_mapRFIDtoNode["0100248599"] = n2; s_mapRFIDtoNode["01002F27BB"] = n3;
    s_mapRFIDtoNode["010023B4DD"] = n4; s_mapRFIDtoNode["010027C273"] = n5;
    s_mapRFIDtoNode["01002F51D2"] = n6; s_mapRFIDtoNode["01002BE8AB"] = n7;
    s_mapRFIDtoNode["010028A404"] = n8; s_mapRFIDtoNode["01002BD6D8"] = n9;
    s_mapRFIDtoNode["01002C28BA"] = n10; s_mapRFIDtoNode["01002A89C5"] = n11;
    s_mapRFIDtoNode["0100250CD9"] = n12; s_mapRFIDtoNode["01002CDED4"] = n13;
    s_mapRFIDtoNode["0100285093"] = n14; s_mapRFIDtoNode["010022B278"] = n15;
    s_mapRFIDtoNode["0100279BC4"] = n16; s_mapRFIDtoNode["01002646EA"] = n17;
    s_mapRFIDtoNode["0100238BF3"] = n18; s_mapRFIDtoNode["01002101B5"] = n18;
    s_mapRFIDtoNode["010023693A"] = n19; s_mapRFIDtoNode["01002A5EB8"] = n19;
    s_mapRFIDtoNode["0100207C40"] = n20; s_mapRFIDtoNode["01002B9B4F"] = n20;
    s_mapRFIDtoNode["010025562C"] = n21; s_mapRFIDtoNode["01002A31AF"] = n21;
    s_mapRFIDtoNode["0100231B3B"] = n22; s_mapRFIDtoNode["01002B6C12"] = n22;
    s_mapRFIDtoNode["01002AA625"] = n26; s_mapRFIDtoNode["01002197DD"] = n26;
    s_mapRFIDtoNode["010027FA4E"] = n27; s_mapRFIDtoNode["01002B3032"] = n27;
    s_mapRFIDtoNode["010028F854"] = n28; s_mapRFIDtoNode["010021B999"] = n28;
    s_mapRFIDtoNode["010022EAA3"] = n29; s_mapRFIDtoNode["0100221160"] = n29;
    s_mapRFIDtoNode["010024BE91"] = n30; s_mapRFIDtoNode["01002809D7"] = n30;
    s_mapRFIDtoNode["010021E0E8"] = n31; s_mapRFIDtoNode["01002B24BC"] = n32;
    s_mapRFIDtoNode["01002B6843"] = n33; s_mapRFIDtoNode["0100249406"] = n34;
    s_mapRFIDtoNode["010026DB03"] = n35; s_mapRFIDtoNode["01002F1975"] = n23;
    s_mapRFIDtoNode["0100294765"] = n24; s_mapRFIDtoNode["01002E5F74"] = n25;
    s_mapRFIDtoNode["0100212345"] = n36;
#elif VAHAK_FULL_DEMO
    s_mapRFIDtoNode["01002BED61"] = n20;
    s_mapRFIDtoNode["01002BF3D0"] = n19;
    s_mapRFIDtoNode["02002E5D4F"] = n18;
    s_mapRFIDtoNode["9999AAA003"] = n17;
    s_mapRFIDtoNode["9999AAA002"] = n16;
    s_mapRFIDtoNode["9999AAA001"] = n15;
    s_mapRFIDtoNode["9999AAA006"] = n17;
    s_mapRFIDtoNode["9999AAA005"] = n16;
    s_mapRFIDtoNode["9999AAA004"] = n15;
    s_mapRFIDtoNode["02002E54D2"] = n14;
    s_mapRFIDtoNode["02002E46FF"] = n13;
    s_mapRFIDtoNode["02002E5B32"] = n12;
    s_mapRFIDtoNode["LLLLBBB002"] = n11;
    s_mapRFIDtoNode["9999BBB003"] = n10;
    s_mapRFIDtoNode["9999BBB002"] = n9;
    s_mapRFIDtoNode["9999BBB001"] = n8;
    s_mapRFIDtoNode["9999BBB006"] = n10;
    s_mapRFIDtoNode["9999BBB005"] = n9;
    s_mapRFIDtoNode["9999BBB004"] = n8;
    s_mapRFIDtoNode["LLLLBBB001"] = n7;
    s_mapRFIDtoNode["02002E5117"] = n6;
    s_mapRFIDtoNode["02002E4DEC"] = n5;
    s_mapRFIDtoNode["02002E6358"] = n4;
    s_mapRFIDtoNode["01002B48AA"] = n3;
    s_mapRFIDtoNode["02002E5028"] = n2;
    s_mapRFIDtoNode["02002E54A5"] = n1;
    s_mapRFIDtoNode["02002E4AB4"] = n0;
#elif SHOPFLOOR_DEMO_SMALL
    s_mapRFIDtoNode["01002BF3D0"] = n0; s_mapRFIDtoNode["01002BED61"] = n1;
    s_mapRFIDtoNode["01002B8059"] = n2; s_mapRFIDtoNode["0100272B26"] = n3;
    s_mapRFIDtoNode["01002B8BD3"] = n4; s_mapRFIDtoNode["0100275FE4"] = n5;
    s_mapRFIDtoNode["0300FF794D"] = n6; s_mapRFIDtoNode["01002B76CC"] = n7;
    s_mapRFIDtoNode["55004EC9F9"] = n8; s_mapRFIDtoNode["03009AB310"] = n9;
    s_mapRFIDtoNode["01002B48AA"] = n10; s_mapRFIDtoNode["01002B8BCE"] = n11;
    s_mapRFIDtoNode["01002BE926"] = n12; s_mapRFIDtoNode["01002BD622"] = n13;
    s_mapRFIDtoNode["01002BBD98"] = n14;
#elif WAREHOUSE_DEMO
    s_mapRFIDtoNode["R1"] = n0; s_mapRFIDtoNode["R2"] = n1;
    s_mapRFIDtoNode["R3"] = n2; s_mapRFIDtoNode["R4"] = n3;
    s_mapRFIDtoNode["R5"] = n4; s_mapRFIDtoNode["R6"] = n5;
#elif SHOPFLOOR_INTERNAL
    s_mapRFIDtoNode["01002BF3D0"] = n22;
    s_mapRFIDtoNode["01002BED61"] = n21;
    s_mapRFIDtoNode["01002B8059"] = n20;
    s_mapRFIDtoNode["0100272B26"] = n19;
    s_mapRFIDtoNode["01002B8BD3"] = n18;
    s_mapRFIDtoNode["0100275FE4"] = n17;
    s_mapRFIDtoNode["01002BBD98"] = n16;
    s_mapRFIDtoNode["0300FF794D"] = n15;
    s_mapRFIDtoNode["03009B720A"] = n14;
    s_mapRFIDtoNode["01002BD622"] = n13;
    s_mapRFIDtoNode["01002B48AA"] = n12;
    s_mapRFIDtoNode["01002B76CC"] = n11;
    s_mapRFIDtoNode["01002BA950"] = n10;
    s_mapRFIDtoNode["02009B48AC"] = n9;
    s_mapRFIDtoNode["010026F95B"] = n8;
    s_mapRFIDtoNode["01002BEB4C"] = n7;
    s_mapRFIDtoNode["0300FF5996"] = n6;
    s_mapRFIDtoNode["0300FF5AF2"] = n5;
    s_mapRFIDtoNode["9999AAA003"] = n4;
    s_mapRFIDtoNode["9999AAA002"] = n3;
    s_mapRFIDtoNode["9999AAA001"] = n2;
    s_mapRFIDtoNode["55004F2ACA"] = n1;
    s_mapRFIDtoNode["03009B614A"] = n0;
#endif
}

//Show approved time plan
void TrafficManager::show_approved_timeplan()
{
    BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl;
    for (unsigned int i = 0; i < approved_time_plan.size(); i++) {
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Agent " << i << " ";
        for (unsigned int j = 0; j < approved_time_plan[i].size(); j++) {
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << approved_time_plan[i][j] << " ";
        }
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl;
    }
}

void TrafficManager::show_approved_plan()
{
    BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Approved Path plan " << endl;
    for (unsigned int i = 0; i < path_list.size(); i++) {
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Agent " << i << " ";
        for (unsigned int j = 0; j < path_list[i].size(); j++) {
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << path_list[i][j] << " ";
        }
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl;
    }
}

void my_handler(int s){
    exit(1);
}

int TrafficManager::overlap_check(int agent_pos, int agent_to_add_pos)
{
    //returns 1 for collision 0 for no collision
    //check position first
    if (agent_pos == -1)
        return 0;
    if (agent_pos == agent_to_add_pos)
        return 1;

    //check agent_pos with adjacent nodes of AGVs already added
    for (unsigned int i = 0; i < TrafficManager::adjacent_nodes[agent_pos].size(); i++)
    {
        if (agent_to_add_pos == TrafficManager::adjacent_nodes[agent_pos][i])
        {
            return 1;//collision
        }
    }

    //check agent_pos with adjacent edges of AGVs already added
    for (unsigned int i = 0; i < TrafficManager::adjacent_edges[agent_pos].size(); i++)
    {
        if (agent_to_add_pos == TrafficManager::adjacent_edges[agent_pos][i])
        {
            return 1;//collision
        }
    }

    return 0;
}

//This will change to incorporate a larger zone around the AGV.
struct collision_details TrafficManager::check_collision(vector<int> time_plan_part, vector< vector<int> > approved_time_plan,\
                                                         int agent_to_add, QVariantList bin_id_list)
{
    //Currently head to head collisions are modelled explicitly cause of corner case
    unsigned int time_step = 1;
    struct collision_details coll;
    int agent_pos;

    time_step = 1;
    for (vector<int>::iterator it=time_plan_part.begin(); it<time_plan_part.end(); it++, time_step++)
    {
        if (time_step > TIME_WINDOW)
        {
            break;
        }
        for (unsigned int i = 0; i < approved_time_plan.size();i++)
        {
            if (i == agent_to_add) {
                continue;
            }
            vector<int> checker = approved_time_plan.at(i);
            if (time_step > checker.size()) {
                //if no future plan is to be added then take the last position
                int stationary_moving;//0 is stationary 1 is moving
                if (bin_id_list[i] =="")
                {
                    stationary_moving = 0;
                }
                else
                {
                    stationary_moving = 1;
                }
                int stationary_time = STATIC_WAIT_TIME;

                //If AGV is stopped at Pickup or Replenish node
                if (checker.at(checker.size()-1) == PICKUP_NODE_NUM || checker.at(checker.size()-1) == REPLENISH_NODE_NUM || \
                        checker.at(checker.size()-1) == PICKUP_NODE_NUM+1 || checker.at(checker.size()-1) == REPLENISH_NODE_NUM+1 )
                    stationary_time = PICKUP_REPLENISH_TIME_WAIT;

                if (time_step > checker.size() + stationary_time && stationary_moving == 1)
                {
                    agent_pos = -1; //Stationary AGV with some future plan will have moved by then
                }
                else
                {
                    agent_pos = checker.at(checker.size()-1);
                }
            }
            else
                agent_pos = checker.at(time_step-1);

            int oc = overlap_check(agent_pos, time_plan_part.at(time_step-1));
            if (oc == 0)
            {
                continue;
                //No collision
            }
            else
            {
                //Collision
                coll.coll_yesno = 1;
                coll.node_edge_no = time_plan_part.at(time_step-1);
#if DEBUG_COLLISION
                BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Collision between agent " << i << " and agent_to_add " << agent_to_add << " at position " << agent_pos << " Time step = " << time_step << endl;
#endif
                return coll;
            }
        }
    }
    coll.coll_yesno = 0;
    coll.node_edge_no = 0;
    return coll;
}

void TrafficManager::agv_time_cost_map_preparation(QString agvId)
{
    QSqlQueryModel agvData;
    DbSelectAgv(TrafficManager::db, &agvData, agvId);

    double section_speed;
    speed_agv speed_struct {
        agvData.record(0).value("AgvStraightSpeed").toDouble(), //cmps
                agvData.record(0).value("AgvCurveSpeed").toDouble(), //cmps
                agvData.record(0).value("AgvRackSpeed").toDouble(), //cmps
                agvData.record(0).value("AgvDefaultSpeed").toDouble(), //cmps
    };
    //Update agv_time_cost_map master copy
    for (int row = 0; row < TrafficManager::number_of_nodes; row++)
    {
        for (int col = 0; col < TrafficManager::number_of_nodes; col++)
        {
            if(TrafficManager::visibility_map[row][col] == 1)
            {
                switch(TrafficManager::behave_graph[row][col])
                {
                case center_tape:
                case center_tape1:
                    section_speed = speed_struct.straight_speed_agv;//cmps
                    break;
                case right_fork:
                case left_fork:
                    section_speed = speed_struct.curve_speed_agv;//cmps
                    break;
                case rack:
                case rack1:
                case rack2:
                    section_speed = speed_struct.rack_speed_agv;//cmps
                    break;
                default:
                    section_speed = speed_struct.default_speed;//cmps
                    break;
                    //ladder case to be added
                }
                TrafficManager::agv_time_cost_map[row][col] = (TrafficManager::distance_map[row][col] * 100.0) / section_speed;
            }
            else
            {
                TrafficManager::agv_time_cost_map[row][col] = 0;
            }
        }
    }
}

/*! \brief add_path_to_plan
 *         Computes collision free path for an AGV considering traffic and adds it to approved_time_plan
 *
 *  Use space time dijkstra algorithm to find collision free shortest path.
 *  Returns 1 on succes, 0 on failure.
 *  
 */

int TrafficManager::add_path_to_plan(int source, int next_dest, int agent_id, int extra_edge_to_add, QVariantList bin_id_list, double dist_to_goal_rfid, int last_node, int agv_current_orientation)
{
    struct collision_details coll_detail;
    int standing_still = 0;
    //Make a copy of approved plan where AGV is free and check collision in that
    doubledim_vec copy_approved_time_plan(TrafficManager::approved_time_plan);//using contructor to copy
    vector<int> time_plan_part;
    path_behaviour_and_cost path_cost;

#if 0
    vector<int> path = djik_algo(source,next_dest,TrafficManager::number_of_nodes);

    //Create time plan
#if DEBUG_MRPP
    BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Extra edge to add for " << agent_id << " = " << extra_edge_to_add << endl;
#endif
    vector<int> time_plan_part = path_totimeplan_simulation(path, extra_edge_to_add);

    //search for this agent in approved plan - already done above
    coll_detail = check_collision(time_plan_part, copy_approved_time_plan, agent_id, bin_id_list);


    //if collision replan and send path
    if (coll_detail.coll_yesno == 0) {
        ////fprintf(stderr,"No collision adding the new plan for agent %d\n",agent_id);
    }
#endif

#if SAME_SPEED_SIMULATION
    path_cost = space_time_djik_simulation(source,next_dest,copy_approved_time_plan,TrafficManager::number_of_nodes,\
                                           agent_id, extra_edge_to_add, bin_id_list, agv_current_orientation, last_node);
#else
    //convert distance_map to time_cost_map depending on AGV speeds

    agv_time_cost_map_preparation(agvId[agent_id]);
    path_cost = space_time_djik_realtime(source, next_dest, copy_approved_time_plan, TrafficManager::number_of_nodes,
                                         agent_id, extra_edge_to_add, dist_to_goal_rfid, bin_id_list, agv_current_orientation, last_node);
#endif
    if (path_cost.path.size() != 0)
    {
#if SAME_SPEED_SIMULATION
        time_plan_part = path_totimeplan_simulation(path_cost.path, extra_edge_to_add);
#else
        time_plan_part = path_totimeplan_realtime(path_cost.path, dist_to_goal_rfid, extra_edge_to_add, last_node);
#endif
    }
    if (path_cost.path.size() == 1)
    {
        standing_still = 1;
    }
    // }

    if (time_plan_part.size() > 0)
    {
        TrafficManager::approved_time_plan[agent_id].resize(0);
        TrafficManager::path_list[agent_id].resize(0);

        //path and time_plan_part now contains collision free path - add this to the back of the vector in approved time plan
        for(vector<int>::iterator it = time_plan_part.begin(); it<time_plan_part.end(); it++)
        {
            TrafficManager::approved_time_plan[agent_id].push_back(*it);
        }
        for(vector<int>::iterator it = path_cost.path.begin(); it<path_cost.path.end(); it++)
        {
            TrafficManager::path_list[agent_id].push_back(*it);
        }
        TrafficManager::turnaround_at_src[agent_id] = path_cost.behaviour[0];//Most current turnaround
        if (standing_still == 0)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

/*! \brief path_totimeplan_realtime
 *         Converts a raw node path to time plan for real time mode.
 *
 *  If AGV path is node 1 -> 2 -> 3
 *  Generates Time plan which indicates at what time instant AGV is at what location.
 *  Includes nodes and edges.
 *  Returns vector containing time plan for an AGV
 *
 */
vector<int> TrafficManager::path_totimeplan_realtime(vector<int> path_toconvert, double dist_to_next_rfid, int extra_edge_to_add, int last_node)
{
    int pause_time_insecs = 4;
    vector<int> time_path;
    vector<int> srcs;
    int link_no = 0;
    int previous_node = path_toconvert.front();
    int next_node = path_toconvert.front();
    int dest = path_toconvert.back();

    if (dist_to_next_rfid != 0.0 && extra_edge_to_add != -1)
    {
        double time_to_next_rfid = (TrafficManager::agv_time_cost_map[last_node][next_node])*(dist_to_next_rfid/TrafficManager::dist_between_rfid[last_node][next_node]);//in seconds
        double fractpart, time_intpart;
        fractpart = modf (time_to_next_rfid, &time_intpart);
        if (fractpart > 0.5)
        {
            time_intpart +=1;
        }
        if (fractpart > 0.2 && time_intpart == 0.0)
        {
            time_intpart = 1;
        }
        for (int index = 0; index < time_intpart; index++)
        {
            time_path.push_back(extra_edge_to_add);
        }
    }

    int time_steps_at_start = 1;
    int time_steps_at_node = 1;
    int time_steps_at_end = 2;

    for (int i=1;i <= time_steps_at_start;i++)
    {
        time_path.push_back(path_toconvert.front());
    }
    path_toconvert.erase(path_toconvert.begin());

    while(1)
    {
        if (previous_node == dest)
        {
            break;
        }
        next_node = path_toconvert.front();
        if (next_node == previous_node)
        {
            for (int i=0; i<pause_time_insecs; i++)
            {
                time_path.push_back(next_node);
            }
            path_toconvert.erase(path_toconvert.begin());
            continue;
        }

        int link_number = TrafficManager::link_no[previous_node][next_node];
        //int link_number = -1;
        double time_steps_at_link = TrafficManager::agv_time_cost_map[previous_node][next_node];
        double fractpart, time_intpart;
        fractpart = modf (time_steps_at_link, &time_intpart);
        if (fractpart > 0.5)
        {
            time_intpart += 1;
        }
        for (int i=0; i<time_intpart; i++)
        {
            time_path.push_back(link_number);
        }

        //destination and other nodes are handled differently
        if (next_node == dest)
        {
            for (int i=0; i < time_steps_at_end; i++)
            {
                time_path.push_back(next_node);
            }
        }
        else
        {
            for (int i=0; i < time_steps_at_node; i++)
            {
                time_path.push_back(next_node);
            }
        }
        path_toconvert.erase(path_toconvert.begin());
        previous_node = next_node;
    }
    return time_path;
#if 0
    printf("Time Path = \n");
    for (int index = 0; index < time_path.size(); index++)
    {
        printf("%d\n",time_path[index]);
    }
#endif
}

/*! \brief path_totimeplan_simulation
 *         Converts a raw node path to time plan in simulation mode.
 *
 *  If AGV path is node 1 -> 2 -> 3
 *  Generates Time plan which indicates at what time instant AGV is at what location.
 *  Includes nodes and edges.
 *
 */
vector<int> TrafficManager::path_totimeplan_simulation(vector<int> path_toconvert, int extra_edge_to_add)
{
    vector<int> time_path;
    int time_steps_at_start = 1;

    vector<int> srcs;
    int time_steps_at_node = 1;
    int time_steps_at_end = 1;
    int time_steps_at_link = 3;
    int previous_node = path_toconvert.front();
    int next_node = path_toconvert.front();
    int link_number = 0;
    int dest = path_toconvert.back();

    if (extra_edge_to_add != -1)
    {
        for(int i = 1; i <= (time_steps_at_link/2); i++)
        {
            time_path.push_back(extra_edge_to_add);
        }
    }

    for (int i=1;i<=time_steps_at_start;i++)
    {
        time_path.push_back(path_toconvert.front());
    }

    path_toconvert.erase(path_toconvert.begin());

    while(1)
    {
        if (previous_node == dest)
            break;
        next_node = path_toconvert.front();
        if (next_node == previous_node) {
            for (int i=0; i<=PAUSE_TIME_STEPS - 1; i++) {
                time_path.push_back(next_node);
            }
            path_toconvert.erase(path_toconvert.begin());
            continue;
        }

        link_number = TrafficManager::link_no[previous_node][next_node];

        for (int i=1; i<=time_steps_at_link; i++)
            time_path.push_back(link_number);

        //destination and other nodes are handled differently
        if (next_node == dest) {
            for (int i=1;i<=time_steps_at_end;i++)
                time_path.push_back(next_node);
        }
        else {
            for (int i=1;i<=time_steps_at_node;i++)
                time_path.push_back(next_node);
        }

        path_toconvert.erase(path_toconvert.begin());
        previous_node = next_node;
    }
    return time_path;
}

int in_assigned_srcs(int iter_src, vector<int> assigned_srcs)
{
    for (unsigned int i = 0; i < assigned_srcs.size();i++) {
        if (iter_src == assigned_srcs[i]) {
            return 1;
        }
    }
    return 0;
}

int in_assigned_dests(int iter_dest, vector<int> assigned_dests)
{
    for (unsigned int i = 0; i < assigned_dests.size();i++)
    {
        if (iter_dest == assigned_dests[i])
        {
            return 1;
        }
    }
    return 0;
}

/*! \brief calc_costs_to_binlocs_andassign
 *         Assigns AGV to bin after calculating Matrix (Free AGV vs Bins)
 *
 *  Calculates traffic cost of every free AGV to reach every scheduled bin and selects lowest cost AGV - bin combination.
 *  Keep assigning AGVs to bins as long as scheduled unassigned bins exist.
 *
 */
void TrafficManager::calc_costs_to_binlocs_andassign(vector<int> srcs, vector<int> dests, QVariantList free_agv_list, QVariantList bin_ids_tosched, vector<int> free_agv, QVariantList bin_id_list)
{
    doubledim_vec agent_binloc_cost_matrix;

    agent_binloc_cost_matrix.resize(srcs.size()); //one row for every agent
    for (unsigned int i = 0; i < srcs.size(); i++)
    {
        agent_binloc_cost_matrix[i].resize(dests.size());
        for (unsigned int j = 0; j < dests.size(); j++)
        {
            agent_binloc_cost_matrix[i][j] = -1;//Initialize matrix to -1
        }
    }

    //Return cost to destination considering traffic
    //First consider simple Dijkstra cost - check collision with higher priority ones
    //If collision try a space time djik and check_collision return
    //If no path return a -1 in the cost matrix
    struct collision_details coll_detail;
    vector<int> time_plan_part;
    path_behaviour_and_cost path_cost;
    for (unsigned int iter_dest = 0; iter_dest < dests.size(); iter_dest++)
    {
        for (unsigned int iter_src = 0; iter_src < srcs.size(); iter_src++)
        {
#if DEBUG_MRPP_BINS
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Bin cost calculating path between " << srcs[iter_src] << " and " << dests[iter_dest] << endl << endl;
#endif
            path_cost = djik_algo(srcs[iter_src],dests[iter_dest],TrafficManager::number_of_nodes);

            if (path_cost.path.size() == 1) {
                agent_binloc_cost_matrix[iter_src][iter_dest] = 0;
#if DEBUG_MRPP_BINS
                BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Bin " <<  bin_ids_tosched[iter_dest].toString().toStdString() << "is where AGV " << srcs[iter_src] << " is" << endl;
#endif
                continue;
            }

            if (path_cost.path.size() == 0) {
                agent_binloc_cost_matrix[iter_src][iter_dest] = -1;
                //fprintf(stderr,"No path between %d and %d\n",dests[iter_dest],srcs[iter_src]);
                continue;
            }

            //Create time plan
#if SAME_SPEED_SIMULATION
            time_plan_part = path_totimeplan_simulation(path_cost.path,-1);
#else
            time_plan_part = path_totimeplan_realtime(path_cost.path, 0.0, -1, srcs[iter_src]);
#endif

            //Make a copy of approved plan where AGV is free and check collision in that
            doubledim_vec copy_approved_time_plan(TrafficManager::approved_time_plan);//using contructor to copy

            copy_approved_time_plan.erase(copy_approved_time_plan.begin()+free_agv[iter_src]);
            coll_detail = check_collision(time_plan_part, copy_approved_time_plan, free_agv[iter_src], bin_id_list);
            //printf("Continuing\n");

            //if collision replan and send path
            if (coll_detail.coll_yesno == 0) {
#if DEBUG_MRPP_BINS
                BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "No collision adding cost of path for agent " << free_agv[iter_src] << endl;
#endif
            }
            else {
                time_plan_part.clear();
                //path_cost.clear();
#if SAME_SPEED_SIMULATION
                path_cost = space_time_djik_simulation(srcs[iter_src],dests[iter_dest],copy_approved_time_plan,\
                                                       TrafficManager::number_of_nodes, free_agv[iter_src],-1, bin_id_list,0,srcs[iter_src]);
#else
                //Real time one
                //Get speeds for one AGV and do the needful
                //convert distance_map to time_cost_map depending on AGV speed.
                agv_time_cost_map_preparation(TrafficManager::agvId[free_agv[iter_src]]);
                path_cost = space_time_djik_realtime(srcs[iter_src], dests[iter_dest], copy_approved_time_plan, TrafficManager::number_of_nodes,\
                                                     free_agv[iter_src], -1, 0.0, bin_id_list, 0, srcs[iter_src]);
#endif
            }

            double cost_part;
#if SAME_SPEED_SIMULATION
            cost_part = path_cost.cost;
#else
            cost_part = path_cost.cost;
#endif
            if (cost_part == -1) {
                agent_binloc_cost_matrix[iter_src][iter_dest] = -1;
                continue;
            }
            else
            {
                agent_binloc_cost_matrix[iter_src][iter_dest] = cost_part;
            }
#if DEBUG_MRPP_BINS
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Cost to pick " << dests[iter_dest] << " bin by " << free_agv[iter_src] << " agent is " << agent_binloc_cost_matrix[iter_src][iter_dest] << endl;
#endif
        }
    }

    //Use agent_binloc_cost_matrix to assign AGVs to bins
#if DEBUG_MRPP_BINS
    BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl << endl << "!!!Adding free agent tasks Now!!!" << endl << endl;
#endif

#if DEBUG_MRPP_BINS
    BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Viewing cost matrix" << endl;
    for (unsigned int i = 0; i < agent_binloc_cost_matrix.size(); i++)
    {
        for (unsigned int j = 0; j < agent_binloc_cost_matrix[i].size(); j++)
        {
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Cost agent " << free_agv[i] << " at location " << srcs[i] << " bin " << j << " bin location " << dests[j] << " is " << agent_binloc_cost_matrix[i][j] << endl;
        }
    }
#endif

    int min_cost,min_row,min_col,agent_id;
    vector<int> assigned_dests, assigned_srcs;
    QList<QVariantList> AgvBinInfo;
    QVariantList agvid, binid;

    while(1)
    {
        min_cost = -1; min_row = -1; min_col = -1;
        for (unsigned int iter_dest = 0; iter_dest< dests.size(); iter_dest++)
        {
            if (in_assigned_dests(iter_dest,assigned_dests))
            {
                continue;
            }
            for (unsigned int iter_src = 0; iter_src < srcs.size(); iter_src++)
            {
                if (in_assigned_srcs(free_agv[iter_src],assigned_srcs))
                    continue;
                if (agent_binloc_cost_matrix[iter_src][iter_dest] == -1)
                {
                    continue;
                }
                else if (min_cost == -1) {
                    min_cost = agent_binloc_cost_matrix[iter_src][iter_dest];
                    min_row = iter_src;
                    min_col = iter_dest;
                }
                else
                {
                    if (min_cost > agent_binloc_cost_matrix[iter_src][iter_dest])
                    {
                        min_cost = agent_binloc_cost_matrix[iter_src][iter_dest];
                        min_row = iter_src;
                        min_col = iter_dest;
                    }
                }
            }
        }

        if(min_cost == -1)
        {
            break;
        }
        else
        {
            assigned_srcs.push_back(free_agv[min_row]);
            assigned_dests.push_back(min_col);
            agent_id = free_agv[min_row];
#if DEBUG_MRPP
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Assigning agent " << agent_id << " to bin " << min_col << endl;
#endif
            //Assign DB query
            //DbInsertAgvBin(TrafficManager::db, QList<QVariantList> list);
            agvid << free_agv_list[min_row];
            binid << bin_ids_tosched[min_col];
        }
    }

    AgvBinInfo << agvid << binid;
    DbInsertAgvBin(TrafficManager::db, AgvBinInfo); //assign bins to agvs in AgvBin table
}

/*! \brief read_visibility_map
 *         Called once at start of Traffic Manager to initialize visibility map.
 *
 *  Visibility map is generated after making map with AGVUI.
 *  Returns true on success, false on failure.
 */
bool TrafficManager::read_visibility_map(QString map_files_path)
{
    const string VM_filename = "/userSelectedConnections.txt";
    string map_visibility_filename = map_files_path.toStdString() + VM_filename;
    string map_visibility_filename_usable = map_visibility_filename.substr(7, map_visibility_filename.length()-1);//Remove the 7 long padding in front

    ifstream map_visibility_file(map_visibility_filename_usable.c_str(), ifstream::in);

    if (!map_visibility_file.is_open())
    {
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", critical) << "Visibility Map cannot be opened, MRPP can't proceed!!";
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", critical) << "Tried to open file: " << map_visibility_filename_usable;
        return false;
    }

    /* Read from text file for visibility map */
    int row = 0;
    while(map_visibility_file)
    {
        string check_str;
        if(!getline(map_visibility_file, check_str))
            break;

        TrafficManager::visibility_map.resize(++row);
        istringstream onerow(check_str);
        int col=0;
        while(onerow)
        {
            string one_element;
            if(!getline(onerow, one_element, ','))
            {
                break;
            }
            //cout << "!! one_element " << one_element << endl;
            TrafficManager::visibility_map[row-1].resize(++col);
            if (one_element[0] == '*')
            {
                TrafficManager::visibility_map[row-1][col-1] = atoi((one_element.c_str()+1));
                //cout << TrafficManager::visibility_map[row-1][col-1]  << endl;
            }
            else if (one_element[0] == "")
            {
                TrafficManager::visibility_map[row-1].resize(--col);
            }
            else
            {
                TrafficManager::visibility_map[row-1][col-1] = atoi(one_element.c_str());
                //cout << TrafficManager::visibility_map[row-1][col-1]  << endl;
            }
        }
    }
    //Reduce the map size by 1
    TrafficManager::visibility_map.resize(row-1);
    return true;
}

/*! \brief read_behaviour_map
 *         Called once at start of Traffic Manager to initialize behaviours to go from node A to B
 *
 *  Behaviours are all user selected when making map.
 *  Returns true on success. False on failure.
 *
 */
bool TrafficManager::read_behaviour_map(QString map_files_path)
{
    const string behave_filename = "/userSelectedActions.txt";

    string map_behave_filename = map_files_path.toStdString() + behave_filename;
    string map_behave_filename_usable = map_behave_filename.substr(7, map_behave_filename.length()-1);//Remove the 7 long padding in front

    ifstream map_behave_file(map_behave_filename_usable.c_str(), ifstream::in);

    if (!map_behave_file.is_open())
    {
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", critical) << "Behave Map cannot be opened, MRPP can't proceed!!";
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", critical) << "Tried to open file: " << map_behave_filename_usable;
        return false;
    }

    /* Read from text file for visibility map */
    int row = 0;
    while(map_behave_file)
    {
        string check_str;
        if(!getline(map_behave_file, check_str))
            break;

        TrafficManager::behave_graph.resize(++row);
        istringstream onerow(check_str);
        int col=0;
        while(onerow)
        {
            string one_element;
            if(!getline(onerow, one_element, ','))
            {
                break;
            }
            //cout << "!! one_element " << one_element << endl;
            TrafficManager::behave_graph[row-1].resize(++col);
            if (one_element[0] == '*')
            {
                TrafficManager::behave_graph[row-1][col-1] = agent_behaviour(atoi((one_element.c_str()+1)));
                //cout << TrafficManager::visibility_map[row-1][col-1]  << endl;
            }
            else if (one_element[0] == "")
            {
                TrafficManager::behave_graph[row-1].resize(--col);
            }
            else
            {
                TrafficManager::behave_graph[row-1][col-1] = agent_behaviour(atoi(one_element.c_str()));
            }
        }
    }
    //Reduce the map size by 1
    TrafficManager::behave_graph.resize(row-1);

    //Go row by row and add 6 if second instance of center_tape or rack
    for (int i = 0; i < TrafficManager::behave_graph.size(); i++)
    {
        int count_center = 0;
        int count_rack = 0;
        for (int j = 0; j < TrafficManager::behave_graph[1].size(); j++)
        {
            if (TrafficManager::behave_graph[i][j] == center_tape)
            {
                count_center++;
            }
            if (TrafficManager::behave_graph[i][j] == rack)
            {
                count_rack++;
                if (count_rack == 2)
                {
                    TrafficManager::behave_graph[i][j] = rack1;
                }
                if (count_rack == 3)
                {
                    TrafficManager::behave_graph[i][j] = rack2;
                }
            }

            if (count_center==2)
            {
                TrafficManager::behave_graph[i][j] = center_tape1;
                count_center = 0;
            }
        }
    }

    return true;
}

/*! \brief initialize_static_map_and_traffic
 *         Called once at start of Traffic Manager to initialize static things. See description.
 *
 *  Initializes visibility map, behaviour map, rfid distances, orientations at edge start
 *  and ends, adjacent nodes and edges.
 *  Returns 1 on success, -1 on failure.
 */
int TrafficManager::initialize_static_map_and_traffic(QString map_files_path)
{
    /* Read visibility, behaviours and orientations from UI */
    if(!read_visibility_map(map_files_path))
    {
        return -1;
    }
    if (!read_behaviour_map(map_files_path))
    {
        return -1;
    }

    TrafficManager::number_of_nodes = TrafficManager::visibility_map.size();


    TrafficManager::behave_graph.resize( TrafficManager::number_of_nodes , vector<int>( TrafficManager::number_of_nodes));

#if SHOPFLOOR_DEMO_SMALL
    //    int orientation_at_edge_end[15][15] = {
    //        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//0
    //        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//1
    //        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//2
    //        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//3
    //        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//4
    //        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//5
    //        {0,0,0,0,0,0,0,90,0,0,0,0,0,0,0},//6
    //        {0,0,0,0,0,0,0,0,180,0,0,0,0,0,0},//7
    //        {0,0,0,0,0,0,0,0,0,180,0,0,0,0,0},//8
    //        {0,0,0,0,0,0,0,0,0,0,180,0,0,0,0},//9
    //        {0,0,0,0,0,0,0,0,0,0,0,180,0,0,0},//10
    //        {0,0,0,0,0,0,0,0,0,0,0,0,180,0,0},//11
    //        {0,0,0,0,0,0,0,0,0,0,0,0,0,180,0},//12
    //        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,180},//13
    //        {270,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//14
    //    };
    float dist_between_rfid[15][15] = {
        {0.0,1.96,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},//0
        {0.0,0.0,2.08,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},//1
        {0.0,0.0,0.0,1.93,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},//2
        {0.0,0.0,0.0,0.0,1.85,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},//3
        {0.0,0.0,0.0,0.0,0.0,1.60,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},//4
        {0.0,0.0,0.0,0.0,0.0,0.0,2.45,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},//5
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,2.45,0.0,0.0,0.0,0.0,0.0,0.0,0.0},//6
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,3.70,0.0,0.0,0.0,0.0,0.0,0.0},//7
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.46,0.0,0.0,0.0,0.0,0.0},//8
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.55,0.0,0.0,0.0,0.0},//9
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.50,0.0,0.0,0.0},//10
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.50,0.0,0.0},//11
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,3.77,0.0},//12
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.82},//13
        {1.82,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},//14
    };
    TrafficManager::dist_between_rfid.resize(15);
    for(int i = 0; i < 15; i++)
    {
        TrafficManager::dist_between_rfid[i].resize(15);
        for (int j = 0; j < 15; j++)
        {
            TrafficManager::dist_between_rfid[i][j] = dist_between_rfid[i][j];
        }
    }

    int o_at_edge_start[15][15] = {
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//0
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//1
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//2
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//3
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//4
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//5
        {0,0,0,0,0,0,0,90,0,0,0,0,0,0,0},//6
        {0,0,0,0,0,0,0,0,180,0,0,0,0,0,0},//7
        {0,0,0,0,0,0,0,0,0,180,0,0,0,0,0},//8
        {0,0,0,0,0,0,0,0,0,0,180,0,0,0,0},//9
        {0,0,0,0,0,0,0,0,0,0,0,180,0,0,0},//10
        {0,0,0,0,0,0,0,0,0,0,0,0,180,0,0},//11
        {0,0,0,0,0,0,0,0,0,0,0,0,0,180,0},//12
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,180},//13
        {270,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//14
    };

    int o_at_edge_end[15][15] = {
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//0
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//1
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//2
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//3
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//4
        {0,0,0,0,0,0,90,0,0,0,0,0,0,0,0},//5
        {0,0,0,0,0,0,0,180,0,0,0,0,0,0,0},//6
        {0,0,0,0,0,0,0,0,180,0,0,0,0,0,0},//7
        {0,0,0,0,0,0,0,0,0,180,0,0,0,0,0},//8
        {0,0,0,0,0,0,0,0,0,0,180,0,0,0,0},//9
        {0,0,0,0,0,0,0,0,0,0,0,180,0,0,0},//10
        {0,0,0,0,0,0,0,0,0,0,0,0,180,0,0},//11
        {0,0,0,0,0,0,0,0,0,0,0,0,0,180,0},//12
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,270},//13
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//14
    };

#elif SHOPFLOOR_INTERNAL

    float dist_between_rfid[23][23] = {
        {0.0,1.17,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,2.11,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,2.11,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,2.11,0.0,2.11,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,2.11,0.0,2.11,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.68,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.44,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {1.69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.9,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.02,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.71,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,4.69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,4.79,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.08,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.01,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.75},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,2.46,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.60,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.85,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.93,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,2.08,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.96,0.0},
    };
    TrafficManager::dist_between_rfid.resize(23);
    for(int i = 0; i < 23; i++)
    {
        TrafficManager::dist_between_rfid[i].resize(23);
        for (int j = 0; j < 23; j++)
        {
            TrafficManager::dist_between_rfid[i][j] = dist_between_rfid[i][j];
        }
    }

    int o_at_edge_start[23][23] = {
        {0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,270,270,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    };

    int o_at_edge_end[23][23] = {
        {0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,270,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    };


#elif EXPO_DEMO

    float dist_between_rfid[28][28] = {
        {0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0,1},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1,0.0},
    };

    TrafficManager::dist_between_rfid.resize(28);
    for(int i = 0; i < 28; i++)
    {
        TrafficManager::dist_between_rfid[i].resize(28);
        for (int j = 0; j < 28; j++)
        {
            TrafficManager::dist_between_rfid[i][j] = dist_between_rfid[i][j];
        }
    }

    int o_at_edge_start[28][28] = {
        {0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //0
        {0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //1
        {0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //2
        {0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //3
        {0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //4
        {0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //5
        {0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //6
        {0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //7
        {0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //8
        {0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //9
        {0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //10
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0}, //11
        {0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //12
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0}, //13
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0}, //14
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0}, //15
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0}, //16
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0}, //17
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0}, //18
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0}, //19
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0}, //20
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0}, //21
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0}, //22
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0}, //23
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0}, //24
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0}, //25
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,270}, //26
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0}, //27
    };

    int o_at_edge_end[28][28] = {
        {0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //0
        {0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //1
        {0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //2
        {0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //3
        {0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //4
        {0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //5
        {0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //6
        {0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //7
        {0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //8
        {0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //9
        {0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //10
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0}, //11
        {0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //12
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0}, //13
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0}, //14
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0}, //15
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0}, //16
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0}, //17
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0}, //18
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0},
    };

#elif VAHAK_FULL_DEMO

    float dist_between_rfid[21][21] = {
        {0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0},
        {0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
    };
    TrafficManager::dist_between_rfid.resize(21);
    for(int i = 0; i < 21; i++)
    {
        TrafficManager::dist_between_rfid[i].resize(21);
        for (int j = 0; j < 21; j++)
        {
            TrafficManager::dist_between_rfid[i][j] = dist_between_rfid[i][j];
        }
    }

    int o_at_edge_start[21][21] = {
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //0
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //1
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //2
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //3
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //4
        {0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0,0,0,0,0}, //5
        {0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //6
        {0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0}, //7
        {0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0}, //8
        {0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0}, //9
        {0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0}, //10
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0}, //11
        {0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0}, //12
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0}, //13
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0}, //14
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0}, //15
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0}, //16
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0}, //17
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0}, //18
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180}, //19
        {0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //20
    };

    int o_at_edge_end[21][21] = {
        {0,0,0,0,0,270,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //0
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //1
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //2
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //3
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //4
        {0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0}, //5
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //6
        {0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0}, //7
        {0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0}, //8
        {0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0}, //9
        {0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0}, //10
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0}, //11
        {0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0}, //12
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0}, //13
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0}, //14
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0}, //15
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0}, //16
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0}, //17
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0}, //18
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180}, //19
        {0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //20
    };

#elif FULL_DEMO
    int o_at_edge_end[37][37] = {
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//3
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//4
        {0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//5
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//6
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//7
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//8
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//9
        {180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//10
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0},//11
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//12
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0},//13
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//14
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,270,0},//15
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0},//16
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,180,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0},//17
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//18
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//19
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//20
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//21
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0},//22
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0,0,0,0,0,0,0,0,0},//23
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0},//24
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0},//25
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0},//26
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0},//27
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0},//28
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0},//29
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0},//30
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0},//31
        {0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//32
        {0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0},//33
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,270,0},//34
        {0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//36
    };

    int o_at_edge_start[37][37] =
    {
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//4
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//5
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//6
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//7
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//8
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//9
        {180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//10
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0},//11
        {0,0,0,0,270,0,0,0,0,270,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//12
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//13
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//14
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,180,0},//15
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0},//16
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0},//17
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//18
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//19
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//20
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//21
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0},//22
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0},//23
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,270,0,0},//24
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0},//25
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0},//26
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0},//27
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0},//28
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0},//29
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0},//30
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0},//31
        {0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//32
        {0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0},//33
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,270,0},//34
        {0,0,0,0,0,0,0,0,0,0,0,0,270,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//35
        {0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//36
    };
#elif WAREHOUSE_DEMO
    float dist_between_rfid[6][6] = {//TBF
        {0.0,1.96,0.0,0.0,0.0,0.0},//0
        {0.0,0.0,2.08,0.0,0.0,0.0},//1
        {0.0,0.0,0.0,1.93,0.0,0.0}, //2
        {0.0,0.0,0.0,0.0,1.85,0.0}, //3
        {0.0,0.0,0.0,0.0,0.0,1.60}, //4
        {0.0,0.0,0.0,0.0,0.0,0.0}, //5
    };
    TrafficManager::dist_between_rfid.resize(6);
    for(int i = 0; i < 6; i++)
    {
        TrafficManager::dist_between_rfid[i].resize(6);
        for (int j = 0; j < 6; j++)
        {
            TrafficManager::dist_between_rfid[i][j] = dist_between_rfid[i][j];
        }
    }
    int o_at_edge_start[6][6] = {//TBF
        {0,0,0,0,0,0},//0
        {180,0,0,0,0,0},//1
        {0,180,0,0,0,0},//2
        {0,0,180,0,0,0},//3
        {0,0,0,180,0,0},//4
        {0,0,0,0,180,0},//5
    };

    int o_at_edge_end[6][6] = {//TBF
        {0,0,0,0,0,0},//0
        {180,0,0,0,0,0},//1
        {0,180,0,0,0,0},//2
        {0,0,180,0,0,0},//3
        {0,0,0,180,0,0},//4
        {0,0,0,0,180,0},//5
    };

#endif

    TrafficManager::orientation_at_edge_start.resize(TrafficManager::number_of_nodes);
    TrafficManager::orientation_at_edge_end.resize(TrafficManager::number_of_nodes);
    for (int i = 0; i < TrafficManager::number_of_nodes; i++)
    {
        TrafficManager::orientation_at_edge_start[i].resize(TrafficManager::number_of_nodes);
        TrafficManager::orientation_at_edge_end[i].resize(TrafficManager::number_of_nodes);
        for(int j = 0; j < TrafficManager::number_of_nodes; j++)
        {
            TrafficManager::orientation_at_edge_start[i][j] = o_at_edge_start[i][j];
            TrafficManager::orientation_at_edge_end[i][j] = o_at_edge_end[i][j];
        }
    }

#if AUTO_CHARGING
    TrafficManager::no_of_charging_stations = 1;
    /* Adding charging stations */
    TrafficManager::charging_stations.resize(TrafficManager::no_of_charging_stations);
    charging_stations[0].node_no = 18;
    charging_stations[0].rfid = "9999AAA005";
    charging_stations[0].free = true;
//    charging_stations[1].node_no = 36;
//    charging_stations[1].rfid = "0100212345";
//    charging_stations[1].free = true;
#endif
    /* Hard coded path is above */

    //Adding adjacent nodes for all nodes - it is same node only!
    TrafficManager::adjacent_nodes.clear();
    TrafficManager::adjacent_nodes.resize(TrafficManager::number_of_nodes);
    for (int i = 0; i < TrafficManager::number_of_nodes; i++)
    {
        TrafficManager::adjacent_nodes[i].push_back(i);
    }

    TrafficManager::distance_map.resize(TrafficManager::number_of_nodes);
    TrafficManager::agv_time_cost_map.resize(TrafficManager::number_of_nodes);
    /* Initialize distance_map */
    for (int i = 0; i < TrafficManager::number_of_nodes; i++)
    {
        TrafficManager::distance_map[i].resize(TrafficManager::number_of_nodes);
#if LIVE_MODE == 1
        TrafficManager::agv_time_cost_map[i].resize(TrafficManager::number_of_nodes);
#endif
        for(int j = 0; j < TrafficManager::number_of_nodes; j++)
        {
            if (i==j)
            {
                continue;//distance_map value is 0
            }
            else if (TrafficManager::visibility_map[i][j] == 1)
            {
#if FULL_DEMO
                TrafficManager::distance_map[i][j] = EDGE_COST_SIMULATION;
#else
                TrafficManager::distance_map[i][j] = dist_between_rfid[i][j];
#endif
#if LIVE_MODE == 1
                TrafficManager::agv_time_cost_map[i][j] = (dist_between_rfid[i][j] * 100.0) / 20.0;
#endif
                //Climbing nodes for main rack
#if FULL_DEMO
                if ((i == 17 && j == 25) || (i == 25 && j == 17) ||\
                        (i == 23 && j == 31) || (i == 31 && j == 23))
                {
                    //TrafficManager::distance_map[i][j] = 30.0;
                    TrafficManager::distance_map[i][j] = EDGE_COST_SIMULATION;
                }

                //Climbing area for pickup
                if ((i == 3 && j == 2) || (i == 2 && j == 3) ||\
                        (i == 2 && j == 1) || (i == 1 && j == 2))
                {
                    //TrafficManager::distance_map[i][j] = 30.0;
                    TrafficManager::distance_map[i][j] = EDGE_COST_SIMULATION;

                }

                //Climbing area for replenishment
                if ((i == 7 && j == 6) || (i == 6 && j == 7) ||\
                        (i == 7 && j == 8) || (i == 8 && j == 7))
                {
                    //TrafficManager::distance_map[i][j] = 30.0;
                    TrafficManager::distance_map[i][j] = EDGE_COST_SIMULATION;
                }
#endif
            }
            else
            {
                TrafficManager::distance_map[i][j] = -1.0;
#if LIVE_MODE == 1
                TrafficManager::agv_time_cost_map[i][j] = -1.0;
#endif
            }
        }
    }

    TrafficManager::link_no.clear();

    //number edges & add adjacent edges to nodes & adjacent nodes to edges
    TrafficManager::link_no.resize(TrafficManager::number_of_nodes);
    for (int i=0;i < TrafficManager::number_of_nodes; i++)
    {
        TrafficManager::link_no[i].resize(TrafficManager::number_of_nodes);
    }

    int link_offset = 0;
    TrafficManager::adjacent_edges.resize(TrafficManager::number_of_nodes);
    for (int i=0;i < TrafficManager::number_of_nodes; i++)
    {
        for(int j = 0; j < TrafficManager::number_of_nodes; j++)
        {
            TrafficManager::link_no[i][j] = TrafficManager::visibility_map[i][j];
            if (i==j)
            {
                TrafficManager::link_no[i][j] = i;
                continue;
            }
            else if (TrafficManager::visibility_map[i][j] == 1)
            {
                if (TrafficManager::link_no[j][i] > TrafficManager::number_of_nodes)
                {
                    TrafficManager::link_no[i][j] = TrafficManager::link_no[j][i];
                }
                else
                {
                    TrafficManager::link_no[i][j] = TrafficManager::number_of_nodes + link_offset;
                    TrafficManager::adjacent_nodes.resize(adjacent_nodes.size() + 1);
                    TrafficManager::adjacent_nodes[adjacent_nodes.size()-1].push_back(i);
                    TrafficManager::adjacent_nodes[adjacent_nodes.size()-1].push_back(j);
                    TrafficManager::adjacent_edges[i].push_back(link_no[i][j]);
                    TrafficManager::adjacent_edges[j].push_back(link_no[i][j]);
                    link_offset++;
                }
            }
        }
    }

    //Adjacent nodes for nodes done and Adjacent edges for edges left. Adjacent edges for nodes done
    TrafficManager::adjacent_edges.resize(TrafficManager::number_of_nodes + link_offset);
    for (int i = 0; i < TrafficManager::number_of_nodes; i++)
    {
        for (int j = 0; j < TrafficManager::number_of_nodes; j++)
        {
            if (link_no[i][j] > TrafficManager::number_of_nodes)
            {//Adjacent edges stored in adjacent_edges[link_no[i][j]]
                for (int k = 0; k < TrafficManager::adjacent_edges[i].size(); k++)
                {
                    TrafficManager::adjacent_edges[link_no[i][j]].push_back(adjacent_edges[i][k]);
                }
                for (int k = 0; k < TrafficManager::adjacent_edges[j].size(); k++)
                {
                    TrafficManager::adjacent_edges[link_no[i][j]].push_back(adjacent_edges[j][k]);
                }
            }
        }
    }

    //Adjacent nodes for ladder nodes.

#if FULL_DEMO
    TrafficManager::adjacent_nodes[25].push_back(17);
    TrafficManager::adjacent_nodes[25].push_back(18);
    TrafficManager::adjacent_nodes[25].push_back(16);
    TrafficManager::adjacent_nodes[25].push_back(26);

    TrafficManager::adjacent_nodes[17].push_back(18);
    TrafficManager::adjacent_nodes[17].push_back(16);
    TrafficManager::adjacent_nodes[17].push_back(25);
    TrafficManager::adjacent_nodes[17].push_back(26);

    TrafficManager::adjacent_nodes[31].push_back(24);
    TrafficManager::adjacent_nodes[31].push_back(23);
    TrafficManager::adjacent_nodes[31].push_back(22);
    TrafficManager::adjacent_nodes[31].push_back(30);

    TrafficManager::adjacent_nodes[23].push_back(24);
    TrafficManager::adjacent_nodes[23].push_back(22);
    TrafficManager::adjacent_nodes[23].push_back(31);
    TrafficManager::adjacent_nodes[23].push_back(30);

    TrafficManager::adjacent_nodes[16].push_back(17);
    TrafficManager::adjacent_nodes[16].push_back(25);
    TrafficManager::adjacent_nodes[16].push_back(26);
    TrafficManager::adjacent_nodes[16].push_back(18);

    TrafficManager::adjacent_nodes[18].push_back(16);
    TrafficManager::adjacent_nodes[18].push_back(17);
    TrafficManager::adjacent_nodes[18].push_back(25);
    TrafficManager::adjacent_nodes[18].push_back(26);

    TrafficManager::adjacent_nodes[26].push_back(25);
    TrafficManager::adjacent_nodes[26].push_back(16);
    TrafficManager::adjacent_nodes[26].push_back(17);
    TrafficManager::adjacent_nodes[26].push_back(18);

    TrafficManager::adjacent_nodes[30].push_back(31);
    TrafficManager::adjacent_nodes[30].push_back(22);
    TrafficManager::adjacent_nodes[30].push_back(23);
    TrafficManager::adjacent_nodes[30].push_back(24);

    TrafficManager::adjacent_nodes[22].push_back(30);
    TrafficManager::adjacent_nodes[22].push_back(31);
    TrafficManager::adjacent_nodes[22].push_back(23);
    TrafficManager::adjacent_nodes[22].push_back(24);

    TrafficManager::adjacent_nodes[24].push_back(31);
    TrafficManager::adjacent_nodes[24].push_back(22);
    TrafficManager::adjacent_nodes[24].push_back(23);
    TrafficManager::adjacent_nodes[24].push_back(30);

    //  Adjacent nodes for ladder edges
    TrafficManager::adjacent_nodes[TrafficManager::link_no[17][25]].push_back(16);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[17][25]].push_back(26);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[17][25]].push_back(18);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[23][31]].push_back(24);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[23][31]].push_back(30);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[23][31]].push_back(22);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[16][17]].push_back(25);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[16][17]].push_back(18);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[16][17]].push_back(26);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[17][18]].push_back(25);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[17][18]].push_back(26);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[17][18]].push_back(16);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[23][24]].push_back(31);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[23][24]].push_back(30);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[23][24]].push_back(22);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[22][23]].push_back(31);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[22][23]].push_back(30);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[22][23]].push_back(24);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[30][31]].push_back(23);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[30][31]].push_back(24);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[30][31]].push_back(22);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[25][26]].push_back(17);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[25][26]].push_back(18);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[25][26]].push_back(16);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[16][32]].push_back(17);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[16][32]].push_back(18);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[16][32]].push_back(25);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[16][32]].push_back(26);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[18][19]].push_back(17);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[18][19]].push_back(16);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[18][19]].push_back(25);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[18][19]].push_back(26);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[26][27]].push_back(25);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[26][27]].push_back(17);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[26][27]].push_back(16);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[26][27]].push_back(18);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[29][30]].push_back(31);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[29][30]].push_back(22);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[29][30]].push_back(23);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[29][30]].push_back(24);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[21][22]].push_back(31);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[21][22]].push_back(30);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[21][22]].push_back(23);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[21][22]].push_back(24);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[24][34]].push_back(31);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[24][34]].push_back(22);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[24][34]].push_back(23);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[24][34]].push_back(30);

    //Ladder Edges adjacent edges
    TrafficManager::adjacent_edges[TrafficManager::link_no[24][34]].push_back(TrafficManager::link_no[23][31]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[24][34]].push_back(TrafficManager::link_no[22][23]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[24][34]].push_back(TrafficManager::link_no[21][22]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[24][34]].push_back(TrafficManager::link_no[30][31]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[24][34]].push_back(TrafficManager::link_no[29][30]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[29][30]].push_back(TrafficManager::link_no[23][31]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[29][30]].push_back(TrafficManager::link_no[22][23]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[29][30]].push_back(TrafficManager::link_no[23][24]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[29][30]].push_back(TrafficManager::link_no[24][34]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[29][30]].push_back(TrafficManager::link_no[21][22]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[21][22]].push_back(TrafficManager::link_no[23][31]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[21][22]].push_back(TrafficManager::link_no[30][31]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[21][22]].push_back(TrafficManager::link_no[29][30]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[21][22]].push_back(TrafficManager::link_no[23][24]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[21][22]].push_back(TrafficManager::link_no[24][34]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[26][27]].push_back(TrafficManager::link_no[17][25]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[26][27]].push_back(TrafficManager::link_no[16][17]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[26][27]].push_back(TrafficManager::link_no[17][18]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[26][27]].push_back(TrafficManager::link_no[18][19]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[26][27]].push_back(TrafficManager::link_no[16][32]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[18][19]].push_back(TrafficManager::link_no[17][25]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[18][19]].push_back(TrafficManager::link_no[25][26]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[18][19]].push_back(TrafficManager::link_no[26][27]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[18][19]].push_back(TrafficManager::link_no[16][17]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[18][19]].push_back(TrafficManager::link_no[16][32]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[16][32]].push_back(TrafficManager::link_no[17][25]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[16][32]].push_back(TrafficManager::link_no[17][18]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[16][32]].push_back(TrafficManager::link_no[18][19]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[16][32]].push_back(TrafficManager::link_no[25][26]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[16][32]].push_back(TrafficManager::link_no[26][27]);

    TrafficManager::adjacent_edges[25].push_back(TrafficManager::link_no[16][17]);
    TrafficManager::adjacent_edges[25].push_back(TrafficManager::link_no[17][18]);
    TrafficManager::adjacent_edges[25].push_back(TrafficManager::link_no[26][27]);
    TrafficManager::adjacent_edges[25].push_back(TrafficManager::link_no[16][32]);
    TrafficManager::adjacent_edges[25].push_back(TrafficManager::link_no[18][19]);

    TrafficManager::adjacent_edges[17].push_back(TrafficManager::link_no[25][26]);
    TrafficManager::adjacent_edges[17].push_back(TrafficManager::link_no[26][27]);
    TrafficManager::adjacent_edges[17].push_back(TrafficManager::link_no[16][32]);
    TrafficManager::adjacent_edges[17].push_back(TrafficManager::link_no[18][19]);

    TrafficManager::adjacent_edges[31].push_back(TrafficManager::link_no[22][23]);
    TrafficManager::adjacent_edges[31].push_back(TrafficManager::link_no[23][24]);
    TrafficManager::adjacent_edges[31].push_back(TrafficManager::link_no[29][30]);
    TrafficManager::adjacent_edges[31].push_back(TrafficManager::link_no[24][34]);
    TrafficManager::adjacent_edges[31].push_back(TrafficManager::link_no[21][22]);

    TrafficManager::adjacent_edges[23].push_back(TrafficManager::link_no[24][34]);
    TrafficManager::adjacent_edges[23].push_back(TrafficManager::link_no[21][22]);
    TrafficManager::adjacent_edges[23].push_back(TrafficManager::link_no[30][31]);
    TrafficManager::adjacent_edges[23].push_back(TrafficManager::link_no[29][30]);

    TrafficManager::adjacent_edges[24].push_back(TrafficManager::link_no[30][31]);
    TrafficManager::adjacent_edges[24].push_back(TrafficManager::link_no[21][22]);
    TrafficManager::adjacent_edges[24].push_back(TrafficManager::link_no[29][30]);
    TrafficManager::adjacent_edges[24].push_back(TrafficManager::link_no[22][23]);
    TrafficManager::adjacent_edges[24].push_back(TrafficManager::link_no[23][31]);

    TrafficManager::adjacent_edges[22].push_back(TrafficManager::link_no[30][31]);
    TrafficManager::adjacent_edges[22].push_back(TrafficManager::link_no[29][30]);
    TrafficManager::adjacent_edges[22].push_back(TrafficManager::link_no[23][24]);
    TrafficManager::adjacent_edges[22].push_back(TrafficManager::link_no[23][31]);
    TrafficManager::adjacent_edges[22].push_back(TrafficManager::link_no[24][34]);

    TrafficManager::adjacent_edges[30].push_back(TrafficManager::link_no[22][23]);
    TrafficManager::adjacent_edges[30].push_back(TrafficManager::link_no[21][22]);
    TrafficManager::adjacent_edges[30].push_back(TrafficManager::link_no[23][24]);
    TrafficManager::adjacent_edges[30].push_back(TrafficManager::link_no[23][31]);
    TrafficManager::adjacent_edges[30].push_back(TrafficManager::link_no[24][34]);

    TrafficManager::adjacent_edges[16].push_back(TrafficManager::link_no[25][26]);
    TrafficManager::adjacent_edges[16].push_back(TrafficManager::link_no[26][27]);
    TrafficManager::adjacent_edges[16].push_back(TrafficManager::link_no[17][25]);
    TrafficManager::adjacent_edges[16].push_back(TrafficManager::link_no[18][19]);
    TrafficManager::adjacent_edges[16].push_back(TrafficManager::link_no[17][18]);

    TrafficManager::adjacent_edges[26].push_back(TrafficManager::link_no[16][17]);
    TrafficManager::adjacent_edges[26].push_back(TrafficManager::link_no[17][18]);
    TrafficManager::adjacent_edges[26].push_back(TrafficManager::link_no[17][25]);
    TrafficManager::adjacent_edges[26].push_back(TrafficManager::link_no[18][19]);
    TrafficManager::adjacent_edges[26].push_back(TrafficManager::link_no[16][32]);

    TrafficManager::adjacent_edges[18].push_back(TrafficManager::link_no[16][17]);
    TrafficManager::adjacent_edges[18].push_back(TrafficManager::link_no[16][32]);
    TrafficManager::adjacent_edges[18].push_back(TrafficManager::link_no[17][25]);
    TrafficManager::adjacent_edges[18].push_back(TrafficManager::link_no[25][26]);
    TrafficManager::adjacent_edges[18].push_back(TrafficManager::link_no[26][27]);
#elif SHOPFLOOR_DEMO_SMALL
    //For Shopfloor demo add adjacent edges and nodes for rack nodes
    TrafficManager::adjacent_nodes[9].push_back(8);
    TrafficManager::adjacent_nodes[8].push_back(9);
    TrafficManager::adjacent_nodes[8].push_back(10);
    TrafficManager::adjacent_nodes[8].push_back(11);
    TrafficManager::adjacent_nodes[8].push_back(12);

    TrafficManager::adjacent_nodes[9].push_back(8);
    TrafficManager::adjacent_nodes[9].push_back(10);
    TrafficManager::adjacent_nodes[9].push_back(11);
    TrafficManager::adjacent_nodes[9].push_back(12);

    TrafficManager::adjacent_nodes[10].push_back(8);
    TrafficManager::adjacent_nodes[10].push_back(9);
    TrafficManager::adjacent_nodes[10].push_back(11);
    TrafficManager::adjacent_nodes[10].push_back(12);

    TrafficManager::adjacent_nodes[11].push_back(8);
    TrafficManager::adjacent_nodes[11].push_back(9);
    TrafficManager::adjacent_nodes[11].push_back(10);
    TrafficManager::adjacent_nodes[11].push_back(12);

    TrafficManager::adjacent_nodes[12].push_back(8);
    TrafficManager::adjacent_nodes[12].push_back(9);
    TrafficManager::adjacent_nodes[12].push_back(10);
    TrafficManager::adjacent_nodes[12].push_back(11);

    TrafficManager::adjacent_edges[8].push_back(TrafficManager::link_no[9][10]);
    TrafficManager::adjacent_edges[8].push_back(TrafficManager::link_no[10][11]);
    TrafficManager::adjacent_edges[8].push_back(TrafficManager::link_no[11][12]);
    TrafficManager::adjacent_edges[8].push_back(TrafficManager::link_no[12][13]);

    TrafficManager::adjacent_edges[9].push_back(TrafficManager::link_no[10][11]);
    TrafficManager::adjacent_edges[9].push_back(TrafficManager::link_no[7][8]);
    TrafficManager::adjacent_edges[9].push_back(TrafficManager::link_no[11][12]);
    TrafficManager::adjacent_edges[9].push_back(TrafficManager::link_no[12][13]);

    TrafficManager::adjacent_edges[10].push_back(TrafficManager::link_no[7][8]);
    TrafficManager::adjacent_edges[10].push_back(TrafficManager::link_no[8][9]);
    TrafficManager::adjacent_edges[10].push_back(TrafficManager::link_no[11][12]);
    TrafficManager::adjacent_edges[10].push_back(TrafficManager::link_no[12][13]);

    TrafficManager::adjacent_edges[11].push_back(TrafficManager::link_no[7][8]);
    TrafficManager::adjacent_edges[11].push_back(TrafficManager::link_no[9][10]);
    TrafficManager::adjacent_edges[11].push_back(TrafficManager::link_no[12][13]);
    TrafficManager::adjacent_edges[11].push_back(TrafficManager::link_no[8][9]);

    TrafficManager::adjacent_edges[12].push_back(TrafficManager::link_no[7][8]);
    TrafficManager::adjacent_edges[12].push_back(TrafficManager::link_no[8][9]);
    TrafficManager::adjacent_edges[12].push_back(TrafficManager::link_no[9][10]);
    TrafficManager::adjacent_edges[12].push_back(TrafficManager::link_no[10][11]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[7][8]].push_back(TrafficManager::link_no[8][9]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[7][8]].push_back(TrafficManager::link_no[9][10]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[7][8]].push_back(TrafficManager::link_no[10][11]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[7][8]].push_back(TrafficManager::link_no[11][12]);
    //TrafficManager::adjacent_edges[TrafficManager::link_no[7][8]].push_back(TrafficManager::link_no[12][13]);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[7][8]].push_back(9);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[7][8]].push_back(10);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[7][8]].push_back(11);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[7][8]].push_back(12);

    TrafficManager::adjacent_edges[TrafficManager::link_no[8][9]].push_back(TrafficManager::link_no[7][8]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[8][9]].push_back(TrafficManager::link_no[9][10]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[8][9]].push_back(TrafficManager::link_no[10][11]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[8][9]].push_back(TrafficManager::link_no[11][12]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[8][9]].push_back(TrafficManager::link_no[12][13]);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[8][9]].push_back(10);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[8][9]].push_back(11);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[8][9]].push_back(12);

    TrafficManager::adjacent_edges[TrafficManager::link_no[9][10]].push_back(TrafficManager::link_no[7][8]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[9][10]].push_back(TrafficManager::link_no[8][9]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[9][10]].push_back(TrafficManager::link_no[10][11]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[9][10]].push_back(TrafficManager::link_no[11][12]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[9][10]].push_back(TrafficManager::link_no[12][13]);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[9][10]].push_back(8);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[9][10]].push_back(11);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[9][10]].push_back(12);

    TrafficManager::adjacent_edges[TrafficManager::link_no[10][11]].push_back(TrafficManager::link_no[9][10]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[10][11]].push_back(TrafficManager::link_no[11][12]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[10][11]].push_back(TrafficManager::link_no[7][8]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[10][11]].push_back(TrafficManager::link_no[8][9]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[10][11]].push_back(TrafficManager::link_no[12][13]);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[10][11]].push_back(8);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[10][11]].push_back(9);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[10][11]].push_back(12);

    TrafficManager::adjacent_edges[TrafficManager::link_no[11][12]].push_back(TrafficManager::link_no[7][8]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[11][12]].push_back(TrafficManager::link_no[8][9]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[11][12]].push_back(TrafficManager::link_no[9][10]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[11][12]].push_back(TrafficManager::link_no[10][11]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[11][12]].push_back(TrafficManager::link_no[12][13]);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[11][12]].push_back(8);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[11][12]].push_back(9);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[11][12]].push_back(10);

    //TrafficManager::adjacent_edges[TrafficManager::link_no[12][13]].push_back(TrafficManager::link_no[7][8]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[12][13]].push_back(TrafficManager::link_no[8][9]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[12][13]].push_back(TrafficManager::link_no[9][10]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[12][13]].push_back(TrafficManager::link_no[10][11]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[12][13]].push_back(TrafficManager::link_no[11][12]);

    TrafficManager::adjacent_nodes[TrafficManager::link_no[12][13]].push_back(8);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[12][13]].push_back(9);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[12][13]].push_back(10);
    TrafficManager::adjacent_nodes[TrafficManager::link_no[12][13]].push_back(11);

    //extra edges collisions
#if 0
    TrafficManager::adjacent_edges[TrafficManager::link_no[13][14]].push_back(TrafficManager::link_no[12][13]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[13][14]].push_back(TrafficManager::link_no[14][0]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[14][0]].push_back(TrafficManager::link_no[0][1]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[14][0]].push_back(TrafficManager::link_no[13][14]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[0][1]].push_back(TrafficManager::link_no[14][0]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[0][1]].push_back(TrafficManager::link_no[1][2]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[1][2]].push_back(TrafficManager::link_no[2][3]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[1][2]].push_back(TrafficManager::link_no[0][1]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[2][3]].push_back(TrafficManager::link_no[1][2]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[2][3]].push_back(TrafficManager::link_no[3][4]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[3][4]].push_back(TrafficManager::link_no[2][3]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[3][4]].push_back(TrafficManager::link_no[4][5]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[4][5]].push_back(TrafficManager::link_no[3][4]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[4][5]].push_back(TrafficManager::link_no[5][6]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[4][5]].push_back(TrafficManager::link_no[3][4]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[4][5]].push_back(TrafficManager::link_no[5][6]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[5][6]].push_back(TrafficManager::link_no[4][5]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[5][6]].push_back(TrafficManager::link_no[6][7]);

    TrafficManager::adjacent_edges[TrafficManager::link_no[6][7]].push_back(TrafficManager::link_no[5][6]);
    TrafficManager::adjacent_edges[TrafficManager::link_no[6][7]].push_back(TrafficManager::link_no[7][8]);
#endif

#elif WAREHOUSE_DEMO

#endif

    TrafficManager::pickup_node = PICKUP_NODE_NUM;
    TrafficManager::replenish_node = REPLENISH_NODE_NUM;

    Initialize_RFIDtoNode_map();
    Initialize_NodetoRFID_map();

#if BEHAVIOR_CODING
    Initialize_Behavior_enumtostring();
    Initialize_Behaviour_stringtoint();
#endif

    return 1;//success
}

#if LIFTER_LOGIC
/*! \brief lifter_request_func
 *  	   Requests Lift Manager for Lifter configuration for an AGV.
 *
 *  Hard-coded ladder nodes used for lifter logic calculation.
 *  Returns an over-ride destination if lifter request is pending. Else sends -1 if lifter request is approved.
 *
 */

int TrafficManager::lift_request_func(int agvindex, QString last_node_rfid)
{
    //Check for next to next RFID as ladder
    int next_node;
    QList<QVariantList> list_ladder;
    QVariantList rfid, config;
    QSqlQueryModel agvmodel;
    QString next_node_RFID;
    QString current_source_node_RFID;
    QString next_node_Rfid_type;
    QString current_source_node_Rfid_type;
    int ladder_check_node;
    int current_source_node;
    int after_first_ladder_node;
    int override_dest_node, stop_at_ladder = 0, stop_node;
    QString after_first_ladder_node_type, last_node_rfid_type;
#if DEBUG_MRPP_LADDER
    int ladder_no = 0;
#endif

    QList<QVariantList> null_list;
    QString null_string = "";
    QVariantList null_string_list;
    null_string_list << null_string;

    if (TrafficManager::path_list[agvindex].size() > 1)
    {
        next_node = TrafficManager::path_list[agvindex][1];
    }
    else
    {
        next_node = -1;
    }

    current_source_node = TrafficManager::path_list[agvindex][0];
    next_node_RFID = s_mapNodetoRFID[Nodenum(next_node)];
    current_source_node_RFID = s_mapNodetoRFID[Nodenum(current_source_node)];

    DbSelectRfidtype(TrafficManager::db, &agvmodel, next_node_RFID);
    next_node_Rfid_type = agvmodel.record(0).value("RfLocationType").toString();

    DbSelectRfidtype(TrafficManager::db, &agvmodel, current_source_node_RFID);
    current_source_node_Rfid_type = agvmodel.record(0).value("RfLocationType").toString();

    DbSelectRfidtype(TrafficManager::db, &agvmodel,last_node_rfid);
    last_node_rfid_type = agvmodel.record(0).value("RfLocationType").toString();
    int on_edge = 1;
    if (last_node_rfid == current_source_node_RFID)
    {
        on_edge = 0;
    }
    else
    {
        on_edge = 1;
    }

#if DEBUG_MRPP_LADDER
    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl << "For Agv number " << agvindex << " next RFID type = " << next_node_Rfid_type.toStdString() << endl;
    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "For Agv number " << agvindex << " current RFID type = " << current_source_node_Rfid_type.toStdString() << endl;
#endif

   // qDebug() << "12345";
    override_dest_node = -1;

    if ((next_node_Rfid_type == "LADDER")
            && (last_node_rfid_type != "LADDER") && (current_source_node_Rfid_type != "LADDER"))
    {
        qDebug() << "ABCDEF";
            ladder_check_node = next_node;
            QList<QVariantList> list;
            QList<QVariant> lifterId;
            QList<QVariant> targetLevel;
            lifterId << "LF01";
            int targetLifterLevel = -1;
            int lifter_override_dest;

            switch(ladder_check_node)
            {
                case 23 :
                    //get lift to level 1
                    targetLifterLevel = 1;
                    if(last_node_rfid_type == "RACK")
                        lifter_override_dest = 22;
                    else
                        lifter_override_dest = 24;
                    break;
            case 17 :
                targetLifterLevel = 2;
                lifter_override_dest = 16;
                break;
                //get lift to level 2
            case 11 :
                //get lift to level 3
                targetLifterLevel = 3;
                lifter_override_dest = 10;
                break;
            case 5 :
                //get lift to level 4
                targetLifterLevel = 4;
                lifter_override_dest = 4;
                break;
            }

            if(targetLifterLevel >= 0)
            {
                targetLevel << targetLifterLevel;
                list << lifterId << targetLevel;
                DbUpdateLifterTargetLevel(TrafficManager::db, list);
            }
            //Check Current Lift Position. If not same as terget, send overridedest

//            int dbLifterTargetLevel;
            QSqlQueryModel lifterTable;
            DbSelectLifterStatus(TrafficManager::db, &lifterTable, "LF01");
//            dbLifterTargetLevel = lifterTable.record(0).value("CurrentLevel").toInt();
            //qDebug() << "AAAAAAAAA : " << lifterTable.record(0).value("CurrentLevel").toInt();
            if(lifterTable.record(0).value("CurrentLevel").toInt() != targetLifterLevel)
            {
                //Decide override destination;
                return lifter_override_dest;
            }
            else
                return -1;
    }
    else if(last_node_rfid_type == "LADDER")
    {
        if(TrafficManager::path_list[agvindex].size() > 1)
        {
            int nodeToReach;
            int i = 0;
            next_node = TrafficManager::path_list[agvindex][i];
            next_node_RFID = s_mapNodetoRFID[Nodenum(next_node)];
            QSqlQueryModel rfidTable;
            DbSelectRfidtype(TrafficManager::db, &rfidTable, next_node_RFID);
            next_node_Rfid_type = rfidTable.record(0).value("RfLocationType").toString();
            while(next_node_Rfid_type == "LADDER")
            {
                i++;
                next_node = TrafficManager::path_list[agvindex][i];
                next_node_RFID = s_mapNodetoRFID[Nodenum(next_node)];
                QSqlQueryModel rfidTable;
                DbSelectRfidtype(TrafficManager::db, &rfidTable, next_node_RFID);
                next_node_Rfid_type = rfidTable.record(0).value("RfLocationType").toString();
            }
            nodeToReach = TrafficManager::path_list[agvindex][i - 1];
            QList<QVariantList> list;
            QList<QVariant> lifterId;
            QList<QVariant> targetLevel;
            lifterId << "LF01";
            int targetLifterLevel = -1;

            switch(nodeToReach)
            {
                case 23 :
                    //get lift to level 1
                    targetLifterLevel = 1;
                    break;
            case 17 :
                targetLifterLevel = 2;
                break;
                //get lift to level 2
            case 11 :
                //get lift to level 3
                targetLifterLevel = 3;

                break;
            case 5 :
                //get lift to level 4
                targetLifterLevel = 4;

                break;
            }
            if(targetLifterLevel >= 0)
            {
                targetLevel << targetLifterLevel;
                list << lifterId << targetLevel;
                DbUpdateLifterTargetLevel(TrafficManager::db, list);
            }
        }
        return -1;

    }
    else
        return -1;
}
#else
int TrafficManager::lift_request_func(int agvindex, QString last_node_rfid)
{
    return -1;
}

#endif

#if DIVERTER_LOGIC
/*! \brief diverter_request_func 
 *  	   Requests RackManager for diverter configuration for an AGV.       
 *
 *  Hard-coded ladder nodes used for diverter logic calculation.
 *  Returns an over-ride destination if diverter request is pending. Else sends -1 if ladder request is approved.
 *   
 */
int TrafficManager::diverter_request_func(int agvindex, QString last_node_rfid)
{
    //Check for next to next RFID as ladder
    int next_node;
    QList<QVariantList> list_ladder;
    QVariantList rfid, config;
    QSqlQueryModel agvmodel;
    QString next_node_RFID;
    QString current_source_node_RFID;
    QString next_node_Rfid_type;
    QString current_source_node_Rfid_type;
    int ladder_check_node;
    int current_source_node;
    int after_first_ladder_node;
    int override_dest_node, stop_at_ladder = 0, stop_node;
    QString after_first_ladder_node_type, last_node_rfid_type;
#if DEBUG_MRPP_LADDER
    int ladder_no = 0;
#endif

    QList<QVariantList> null_list;
    QString null_string = "";
    QVariantList null_string_list;
    null_string_list << null_string;

    if (TrafficManager::path_list[agvindex].size() > 1)
    {
        next_node = TrafficManager::path_list[agvindex][1];
    }
    else
    {
        next_node = -1;
    }

    current_source_node = TrafficManager::path_list[agvindex][0];
    next_node_RFID = s_mapNodetoRFID[Nodenum(next_node)];
    current_source_node_RFID = s_mapNodetoRFID[Nodenum(current_source_node)];

    DbSelectRfidtype(TrafficManager::db, &agvmodel, next_node_RFID);
    next_node_Rfid_type = agvmodel.record(0).value("RfLocationType").toString();

    DbSelectRfidtype(TrafficManager::db, &agvmodel, current_source_node_RFID);
    current_source_node_Rfid_type = agvmodel.record(0).value("RfLocationType").toString();

    DbSelectRfidtype(TrafficManager::db, &agvmodel,last_node_rfid);
    last_node_rfid_type = agvmodel.record(0).value("RfLocationType").toString();
    int on_edge = 1;
    if (last_node_rfid == current_source_node_RFID)
    {
        on_edge = 0;
    }
    else
    {
        on_edge = 1;
    }

#if DEBUG_MRPP_LADDER
    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl << "For Agv number " << agvindex << " next RFID type = " << next_node_Rfid_type.toStdString() << endl;
    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "For Agv number " << agvindex << " current RFID type = " << current_source_node_Rfid_type.toStdString() << endl;
#endif

    override_dest_node = -1;

    if ((next_node_Rfid_type == "LADDER" || current_source_node_Rfid_type == "LADDER") \
            && (last_node_rfid_type != "LADDER" || on_edge == 0))
    {
        if (current_source_node_Rfid_type == "LADDER")
        {
            ladder_check_node = current_source_node;
            if (TrafficManager::path_list.size() > 1)
            {
                after_first_ladder_node = TrafficManager::path_list[agvindex][1];
            }
            else
            {
                after_first_ladder_node = -1;
                return -1;
            }
        }
        else
        {
            ladder_check_node = next_node;
            if (TrafficManager::path_list.size() > 2)
            {
                after_first_ladder_node = TrafficManager::path_list[agvindex][2];
            }
            else
            {
                after_first_ladder_node = -1;
                return -1;
            }
        }

        switch(ladder_check_node)
        {
        case 14:
#if DEBUG_MRPP_LADDER
            //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Updating ladder 1 with ";
            ladder_no = 1;
#endif
            rfid << s_mapNodetoRFID[Nodenum(ladder_check_node)];
            if (after_first_ladder_node == 15)
            {
                config << "SI";
#if DEBUG_MRPP_LADDER
                //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "SI" << endl;
#endif
            }
//            else if (after_first_ladder_node == 16)
//            {
//                config << "SO";
//#if DEBUG_MRPP_LADDER
//                //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "SO" << endl;
//#endif
//            }
            else //25 and have to go up
            {
                if (last_node_rfid_type == "RACK")
                {
                    config << "SO";
                    stop_at_ladder = 1;
                    stop_node = 17;
#if DEBUG_MRPP_LADDER
                    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "SO" << endl;
#endif
                }
                else
                {
                    config << "I";
#if DEBUG_MRPP_LADDER
                    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "I" << endl;
#endif
                }
            }
            break;
//        case 7:
//            DbSelectRfidtype(TrafficManager::db, &agvmodel, s_mapNodetoRFID[Nodenum(after_first_ladder_node)]);
//            after_first_ladder_node_type = agvmodel.record(0).value("RfLocationType").toString();
//            if (after_first_ladder_node_type != "LADDER")
//            {
//                return -1;
//            }
//            rfid << s_mapNodetoRFID[Nodenum(after_first_ladder_node)];
//#if DEBUG_MRPP_LADDER
//            //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Updating ladder 1 with O" << endl;
//            ladder_no = 1;
//#endif
//            config << "O";
//            stop_at_ladder = 1;
//            stop_node = 17;
//            break;
        case 18:
#if DEBUG_MRPP_LADDER
            //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Updating ladder 2 with ";
            ladder_no = 2;
#endif
            rfid << s_mapNodetoRFID[Nodenum(ladder_check_node)];
            /*if (after_first_ladder_node == 22)
            {
                config << "SI";
#if DEBUG_MRPP_LADDER
                //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "SI" << endl;
#endif
            }
            else*/ \
            if (after_first_ladder_node == 19)
            {
                config << "SO";
#if DEBUG_MRPP_LADDER
                //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "SO" << endl;
#endif
            }
            else // have to go up next is 31
            {
                if (last_node_rfid_type == "RACK")
                {
                    config << "SO";
                    stop_at_ladder = 1;
                    stop_node = 23;
#if DEBUG_MRPP_LADDER
                    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "SO" << endl;
#endif
                }
//                else
//                {
//                    config << "I";
//#if DEBUG_MRPP_LADDER
//                    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "I" << endl;
//#endif
//                }
            }
            break;
//        case 11:
//            DbSelectRfidtype(TrafficManager::db, &agvmodel, s_mapNodetoRFID[Nodenum(after_first_ladder_node)]);
//            after_first_ladder_node_type = agvmodel.record(0).value("RfLocationType").toString();
//            if (after_first_ladder_node_type != "LADDER")
//            {
//                return -1;
//            }
//#if DEBUG_MRPP_LADDER
//            //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Updating ladder 2 with O" << endl;
//            ladder_no = 2;
//#endif
//            rfid << s_mapNodetoRFID[Nodenum(after_first_ladder_node)];
//            config << "O";
//            stop_at_ladder = 1;
//            stop_node = 23;
//            break;
        default:
            return -1;//continue on your way
        }
        //Update in DB
        override_dest_node = ladder_check_node;
        list_ladder << rfid << config;
    }
    else
    {
        if (TrafficManager::agv_request_diverter[agvindex].agv_request_status != NOT_REQUESTED_DIVERTER)
        {
            null_list.clear();
            null_list << TrafficManager::agv_request_diverter[agvindex].diverter_rfid << null_string_list;
            DbUpdateLadderConfig(TrafficManager::db, null_list);
            TrafficManager::agv_request_diverter[agvindex].diverter_rfid.clear();
            TrafficManager::agv_request_diverter[agvindex].agv_request_status = NOT_REQUESTED_DIVERTER;
        }
        return -1;
        //no diverter based traffic planning needs to be done
        //no diverter request needs to be sent
    }

    QSqlQueryModel ladderconfig;
    DbSelectLadderConfig(TrafficManager::db, &ladderconfig, rfid[0].toString());

    if ( TrafficManager::agv_request_diverter[agvindex].agv_request_status == DIVERTER_REQUEST_PENDING \
         && ladderconfig.record(0).value("LadderConfigAck").toInt() == 1 \
         && ladderconfig.record(0).value("LadderConfig").toString() != "")
    {
        //LadderManager has read the Ladder config and got ack from PLC
        if (ladderconfig.record(0).value("LadderConfig").toString() != config[0])
        {   //the config written earlier to PLC is different
            //from one needed now. Request again in next cycle
            TrafficManager::agv_request_diverter[agvindex].agv_request_status = NOT_REQUESTED_DIVERTER;
#if DEBUG_MRPP_LADDER
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "!!!!!!!!!!!!RE-REQUEST TBD !!!!!!!!!!" << endl;
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "AGV: " << agvindex << " Config requested :" << config[0].toString().toStdString();
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << " Config on ladder : " << ladderconfig.record(0).value("LadderConfig").toString().toStdString() << " ladder no.: " << ladder_no << endl;
#endif
        }
        else
        {
            //move the AGV forward
            TrafficManager::agv_request_diverter[agvindex].agv_request_status = DIVERTER_REQUEST_ACCEPTED;
            TrafficManager::agv_request_diverter[agvindex].ladder_config_accepted = ladderconfig.record(0).value("LadderConfig").toString();
#if DEBUG_MRPP_LADDER
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Acked Config : " << ladderconfig.record(0).value("LadderConfig").toString().toStdString() << " AGV: " << agvindex << " ladder no.: " << ladder_no << " Acked " << endl;
#endif
        }
        //clear this request
        null_list << rfid << null_string_list;
        DbUpdateLadderConfig(TrafficManager::db, null_list);
        usleep(1000);
    }
    else if ((TrafficManager::agv_request_diverter[agvindex].agv_request_status == NOT_REQUESTED_DIVERTER) \
             || (TrafficManager::agv_request_diverter[agvindex].ladder_config_accepted != config[0] && \
                 TrafficManager::agv_request_diverter[agvindex].agv_request_status == DIVERTER_REQUEST_ACCEPTED))
    {
        //        null_list << rfid << null_string_list;
        //        DbUpdateLadderConfig(TrafficManager::db, null_list);
        //        usleep(1000);
        TrafficManager::agv_request_diverter[agvindex].diverter_rfid.clear();
        TrafficManager::agv_request_diverter[agvindex].diverter_rfid << rfid;
        DbUpdateLadderConfig(TrafficManager::db, list_ladder);
        TrafficManager::agv_request_diverter[agvindex].agv_request_status = DIVERTER_REQUEST_PENDING;
        TrafficManager::agv_request_diverter[agvindex].ladder_config_accepted = "";
#if DEBUG_MRPP_LADDER
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Requesting Ladder config: " << config[0].toString().toStdString();
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << " for AGV: " << agvindex;
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << " On ladder: " << ladder_no << endl;
#endif
    }

    if (TrafficManager::agv_request_diverter[agvindex].agv_request_status == DIVERTER_REQUEST_ACCEPTED)
    {
        override_dest_node = -1;
    }
    else if (override_dest_node == -1)
    {
        override_dest_node = ladder_check_node;//which is an ladder RFID
    }

    if (stop_at_ladder == 1)
    {
        override_dest_node = stop_node;
    }

    return override_dest_node;
}
#else
int TrafficManager::diverter_request_func(int agvindex, QString last_node_rfid)
{
    return -1;
}
#endif

/*! \brief simulate_goal not being used now.
 *         Delete soon!
 *
 *  Simulates bin unload and load for testing.
 */
void TrafficManager::simulate_goal()
{
    QString dbName = "TrafficManagerSimulatorDb";
    // Connect to the database
    if (!DbCreateConnection(dbName))
    {
        qDebug() << "Traffic Manager Simulator Not connected to DB!";
    }
    qDebug() << "Traffic Manager Simulator Connected to DB!";
    QSqlDatabase tdb = QSqlDatabase::database(dbName);

    sleep(20);

    //Initialize time_plan and path_list
    while(1)
    {
        //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "MRPP: Simulate while " << endl;
        QSqlQueryModel agvTable;
        DbSelectAgvView(tdb, &agvTable);
        QString agvid, behave, bin_status, Bin_associated_with_agv;
        for (int i = 0; i < agvTable.rowCount(); i++)
        {
            Bin_associated_with_agv = agvTable.record(i).value("BinId").toString();
            if(Bin_associated_with_agv != NULL)//bin is associated with AGV
            {
                //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "MRPP: Simulate while bin associated with " << agvid.toStdString() << endl;
                agvid = agvTable.record(i).value("AgvId").toString();
                if (path_list[i].size() > 1)
                {
                    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << agvid.toStdString() << " Reaches node " << path_list[i][0] << endl;
                    //Add Last RFID here
                    QString new_last_RFID = s_mapNodetoRFID[Nodenum(path_list[i][0])];
                    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Position " << new_last_RFID.toStdString() << endl;
                    DbUpdateAgvLocation(tdb, agvid, new_last_RFID);
                    //path_list[i].erase(path_list[i].begin());
                    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << agvid.toStdString() << " going for " << path_list[i][1] << endl;
                }
                if (path_list[i].size() == 1)
                {
                    QString new_last_RFID = s_mapNodetoRFID[Nodenum(path_list[i][0])];
                    qDebug() << "No, this one";
                    DbUpdateAgvLocation(tdb, agvid, new_last_RFID);
                    bin_status = agvTable.record(i).value("AgvBinStatus").toString();
                    QString dest_rfid;
                    if(bin_status == "LOADED")
                        dest_rfid = agvTable.record(i).value("BinTargetLocationId").toString();
                    else
                        dest_rfid = agvTable.record(i).value("BinLocationId").toString();
                    int bin_node = s_mapRFIDtoNode[dest_rfid];
                    int cur_node = s_mapRFIDtoNode[new_last_RFID];

//                    if (path_list[i][0] == PICKUP_NODE_NUM)
//                    {
//                        //set to busy and wait
//                        //                        behave = "BUSY";
//                        //                        DbUpdateAgvCurBehavior(TrafficManager::db, agvid, behave);
//                        //                        //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << agvid.toStdString() << " is at pick station " << endl;
//                        //                        sleep(2);
//                        //                        behave = "FREE";
//                        //                        DbUpdateAgvCurBehavior(TrafficManager::db, agvid, behave);
//                    }
//                    else if  (path_list[i][0] == REPLENISH_NODE_NUM)
//                    {
//                        //set to busy and wait
//                        //                        behave = "BUSY";
//                        //                        DbUpdateAgvCurBehavior(TrafficManager::db, agvid, behave);
//                        //                        //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << agvid.toStdString() << " is at replenish station " << endl;
//                        //                        sleep(2);
//                        //                        behave = "FREE";
//                        //                        DbUpdateAgvCurBehavior(TrafficManager::db, agvid, behave);
//                    }
                    qDebug() <<"Bin Node"<< bin_node <<" Curr Node" << cur_node;
                    if (bin_node == cur_node)
                    {
                        qDebug() <<"Bin Node"<< bin_node <<" Curr Node" << cur_node;
                        //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << agvid.toStdString() << " Has reached bin" << endl;
                        //Depending on bin status load or unload the bin
                        if (bin_status == "EMPTY" && (agvTable.record(i).value("ItemCountToPick").toInt() != 0 || agvTable.record(i).value("ItemCountToPut").toInt() != 0))
                        {
                            //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "MRPP : " << agvid.toStdString() << " Loading bin " << endl;
                            DbAgvLoadsBin(tdb, agvid);


                            QSqlQueryModel rfidTypeTable;
                            DbSelectRfidtype(tdb, &rfidTypeTable, new_last_RFID);

                            if(rfidTypeTable.record(0).value("RfLocationType").toString() == "PICKSTATIONACCESSPOINT")
                            {

#if 1 // added by tanya this is code for unloading bin properly
                                //read database for entries
                                QSqlQueryModel tableAgvView;
                                DbSelectAgvOrderView (tdb, &tableAgvView, agvid, "STORE");

                                if (tableAgvView.rowCount() > 0)
                                {
                                    QVariantList orderList, unitList, partitionList, countCompleteList, totalCountList;
                                    QList<QVariantList> input, input2;
                                    orderList << tableAgvView.record (0).value ("OrderId").toString ();
                                    unitList << tableAgvView.record (0).value ("UnitId").toString ();
                                    partitionList << tableAgvView.record (0).value ("PartitionId").toString ();
                                    countCompleteList << tableAgvView.record (0).value ("ItemCountComplete").toInt () + 1;
                                    totalCountList << (tableAgvView.record (0).value ("ItemCount").toInt () + 1);


                                    input << countCompleteList << orderList << unitList << partitionList;
                                    DbUpdateCompletedUnitCountForOrder(tdb, input);

                                    input2 << partitionList << unitList << totalCountList;
                                    DbUpdateUnitsInPartition(tdb, input2);
                                }
#endif


                            }

                            //update the bin partitions and item count here
                            DbResetBinLocation(tdb, Bin_associated_with_agv);


                        }

                        else if (bin_status == "LOADED" && agvTable.record(i).value("ItemCountToPick").toInt() == 0 && agvTable.record(i).value("ItemCountToPut").toInt() == 0)
                        {
                            //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "MRPP : " << agvid.toStdString() << " Unloading bin " << endl;
                            DbAgvUnloadsBin(tdb, agvid);
                            QSqlQueryModel rfidTypeTable;
                            DbSelectRfidtype(tdb, &rfidTypeTable, dest_rfid);
                            if(rfidTypeTable.record(0).value("RfLocationType").toString() == "PICKSTATIONACCESSPOINT")
                            {
                                DbResetBinLocation(tdb, Bin_associated_with_agv);
                                DbResetBinTargetLocation(tdb, Bin_associated_with_agv);
                                //update here for unloading the bin at PICKSTATIONACCESSPOINT
                                //make it empty here and remove

#if 1 // added by tanya this is code for unloading bin properly
                                //read database for entries
                                QSqlQueryModel tableAgvView;
                                DbSelectAgvOrderView (tdb, &tableAgvView, agvid, "RETRIEVE");

                                if (tableAgvView.rowCount() > 0)
                                {
                                    QVariantList orderList, unitList, partitionList, countCompleteList, totalCountList;
                                    QList<QVariantList> input, input2;
                                    orderList << tableAgvView.record (0).value ("OrderId").toString ();
                                    unitList << tableAgvView.record (0).value ("UnitId").toString ();
                                    partitionList << tableAgvView.record (0).value ("PartitionId").toString ();
                                    countCompleteList << tableAgvView.record (0).value ("ItemCountComplete").toInt ();
                                    totalCountList << (tableAgvView.record (0).value ("ItemCount").toInt () - 1);

                                    input << countCompleteList << orderList << unitList << partitionList;
                                    DbUpdateCompletedUnitCountForOrder(tdb, input);

                                    input2 << partitionList << unitList << totalCountList;
                                    DbUpdateUnitsInPartition(tdb, input2);
                                }
#endif


                            }
                            else
                                DbUpdateBinLocation(tdb, Bin_associated_with_agv, dest_rfid);

                        }
                    }
                }
            }
        }//for
        sleep(5);
    }//while
}


/*! \brief mrpp_main_thread - updates approved time plan for all AGVs every x milliseconds
 *
 *
 *  Mother function of traffic manager.
 *
 */

void TrafficManager::mrpp_main_thread()
{
    TrafficManager::status_loop_started = true;

    QString dbName = "TrafficManagerMainDb";
    // Connect to the database
    if (!DbCreateConnection(dbName))
    {
        qDebug() << "Traffic Manager Not connected to DB!";
    }
    qDebug() << "Traffic Manager Connected to DB!";
    TrafficManager::db = QSqlDatabase::database(dbName);
    vector<int> free_agv;
    QList<QVariantList> ladder_done_list;
    QString currbehave;
    vector<int> agv_current_orientation;

    QSqlQueryModel agvTable_initial;
    DbSelectAgvView(db, &agvTable_initial);//to get number of AGVs

    TrafficManager::agv_request_diverter.resize(agvTable_initial.rowCount());
    vector<int> no_progress_to_goal_counter;
    no_progress_to_goal_counter.resize(agvTable_initial.rowCount());
    vector<int> idle_counter;
    idle_counter.resize(agvTable_initial.rowCount());
    TrafficManager::override_dest.resize(agvTable_initial.rowCount());
    agv_current_orientation.resize(agvTable_initial.rowCount());

    for (int agent_index = 0; agent_index < agvTable_initial.rowCount(); agent_index++)
    {
        no_progress_to_goal_counter[agent_index] = 0;
        idle_counter[agent_index] = 0;
        TrafficManager::agv_request_diverter[agent_index].agv_request_status = NOT_REQUESTED_DIVERTER;
        TrafficManager::override_dest[agent_index].rfid = "";
        TrafficManager::override_dest[agent_index].charging_station_index = -1;
        agv_current_orientation[agent_index] = 0;
    }

    sleep(6);
    //initialize MRPP plans

    TrafficManager::approved_time_plan.clear();
    TrafficManager::path_list.clear();
    TrafficManager::turnaround_at_src.clear();
    TrafficManager::agent_status.clear();
    TrafficManager::approved_time_plan.resize(agvTable_initial.rowCount());
    TrafficManager::path_list.resize(agvTable_initial.rowCount());
    TrafficManager::turnaround_at_src.resize(agvTable_initial.rowCount());
    TrafficManager::agent_status.resize(agvTable_initial.rowCount());

    for (int i = 0; i < agvTable_initial.rowCount(); i++)
    {
        TrafficManager::approved_time_plan[i].resize(1);
        TrafficManager::agent_status[i] = 1;//0 is busy 1 is free
        TrafficManager::path_list[i].resize(1);

        QString last_rfid = agvTable_initial.record(i).value("AgvLastRfId").toString();
        TrafficManager::approved_time_plan[i][0] = s_mapRFIDtoNode[last_rfid];
        TrafficManager::path_list[i][0] = s_mapRFIDtoNode[last_rfid];
        TrafficManager::turnaround_at_src[i] = 0; //No turnaround by default
        TrafficManager::agvId.push_back(agvTable_initial.record(i).value("AgvId").toString());
    }

    while(1)
    {
        if(TrafficManager::status_loop_started)
        {
            usleep(100000);//100ms sleep
            free_agv.clear();

            QSqlQueryModel agvTable;
            DbSelectAgvView(TrafficManager::db, &agvTable);
            QVariantList free_agv_list;
            vector<int> extra_edge_to_add;
            QVariantList bin_id_list;

#if ERASE_PREV_PLAN
            //clear everything in case of memoryless MRPP
            TrafficManager::approved_time_plan.clear();
            TrafficManager::path_list.clear();
            TrafficManager::turnaround_at_src.clear();
            TrafficManager::agent_status.clear();

            //Read current positions of agents
            //Re-Initialize approved_time_plan with current positions
            //TrafficManager::approved_time_plan and agent_status and priority mapping are initialzed here
            TrafficManager::approved_time_plan.resize(agvTable.rowCount());
            TrafficManager::path_list.resize(agvTable.rowCount());
            TrafficManager::turnaround_at_src.resize(agvTable.rowCount());
            TrafficManager::agent_status.resize(agvTable.rowCount());
#endif
            extra_edge_to_add.resize(TrafficManager::number_of_nodes);

            //Initialize time_plan and path_list for all AGVs in system
            for (int i = 0; i < agvTable.rowCount(); i++)
            {
                QString bin_id = agvTable.record(i).value("BinId").toString();
                bin_id_list << bin_id;

#if ERASE_PREV_PLAN//only if prev plan is to be erased
                TrafficManager::approved_time_plan[i].resize(1);
                TrafficManager::agent_status[i] = 1;//0 is busy 1 is free
                TrafficManager::path_list[i].resize(1);
#endif
                QString goal_rfid =  agvTable.record(i).value("AgvGoal").toString();
                QString last_rfid = agvTable.record(i).value("AgvLastRfId").toString();
                int edge_or_node_toupdate;

                if (goal_rfid == "")
                {
                    TrafficManager::approved_time_plan[i][0] = s_mapRFIDtoNode[last_rfid];
                    TrafficManager::path_list[i][0] = s_mapRFIDtoNode[last_rfid];
                    TrafficManager::turnaround_at_src[i] = 0;
                    extra_edge_to_add[i] = -1;
                }
                else
                {
                    if (last_rfid == goal_rfid)
                    {
                        edge_or_node_toupdate = s_mapRFIDtoNode[last_rfid];
                        extra_edge_to_add[i] = -1;
                    }
                    else
                    {
                        int node1 = s_mapRFIDtoNode[last_rfid];
                        int node2 = s_mapRFIDtoNode[goal_rfid];
                        edge_or_node_toupdate = TrafficManager::link_no[node1][node2];
#if DEBUG_MRPP
                        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "For agv " << i << " between nodes " << node1 << "," << node2 << " Edge to add = " << edge_or_node_toupdate << endl;
#endif
                        extra_edge_to_add[i] = edge_or_node_toupdate;
                    }
#if ERASE_PREV_PLAN
                    TrafficManager::path_list[i][0] = edge_or_node_toupdate;
                    TrafficManager::turnaround_at_src[i] = 0;
                    TrafficManager::approved_time_plan[i][0] = edge_or_node_toupdate;
#endif
                }

#if DEBUG_MRPP
                BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "AGV " << i << " static at " << TrafficManager::path_list[i][0] << endl;
#endif
            }

            //Plan collision free path for every AGV in turn
            for(int i =0; i < agvTable.rowCount(); i++)
            {
                QString Can_be_scheduled;
                QString Bin_associated_with_agv;

                int items_to_pick, items_to_replenish, dest_node, source_node, goal_node, last_node;
                int next_next_RFID_node = -1;
                QString agvid, bin_status, binlocationid, last_node_rfid, goal_RFID, next_next_RFID;
                QString dest_rfid, source_rfid;
                QString agv_goal_behaviour;

                Can_be_scheduled = agvTable.record(i).value("AgvTaskStatus").toString();
                agvid = agvTable.record(i).value("AgvId").toString();

                if (Can_be_scheduled == "BUSY")
                {
                    TrafficManager::agent_status[i] = 0;//Busy
                }
                else
                {
                    //By default AGV is free
                    Bin_associated_with_agv = agvTable.record(i).value("BinId").toString();
                    //Check for AGV to be sent to charging
#if AUTO_CHARGING
                    if (Bin_associated_with_agv == NULL && TrafficManager::override_dest[i].rfid == "")
                    {
                        idle_counter[i]++;
                        //Query Last RFID of this AGV
                        if (idle_counter[i] > 100)//> 10s idle and Last RFID is not charge Station
                        {
#ifdef DEBUG_MRPP
                            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Setting out for charging station AGV: " << i ;
#endif
                            //select one charging station to head to
                            //go through list of charging station and select first one that is free
                            for (int charge_station_index = 0; charge_station_index < TrafficManager::no_of_charging_stations; charge_station_index++)
                            {
                                if (charging_stations[charge_station_index].free == true)
                                {
                                    TrafficManager::override_dest[i].rfid = charging_stations[charge_station_index].rfid;
                                    TrafficManager::override_dest[i].charging_station_index = charge_station_index;
                                    charging_stations[charge_station_index].free = false;
                                    idle_counter[i] = 0;
#ifdef DEBUG_MRPP
                                    BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << " Charge station = " << charge_station_index << endl;
#endif
                                    break;
                                }
                            }
                        }
                    }
#endif
                    if(Bin_associated_with_agv != NULL || TrafficManager::override_dest[i].rfid != "")//bin is associated with AGV or an override dest is associated with AGV
                    {
                        last_node_rfid = agvTable.record(i).value("AgvLastRfid").toString();
                        last_node = s_mapRFIDtoNode[last_node_rfid];
                        goal_RFID =  agvTable.record(i).value("AgvGoal").toString();
                        int goal_RFID_node = s_mapRFIDtoNode[goal_RFID];
                        QString agv_goal_behavior = agvTable.record(i).value("AgvGoalBehavior").toString();
                        QString agv_current_behavior = agvTable.record(i).value("AgvCurBehavior").toString();
                        double dist_from_last_rfid = agvTable.record(i).value("DistFromLastRfid").toDouble();

                        //Set Destination first
                        if (Bin_associated_with_agv == NULL)
                        {
                            //Sending AGV to a charging station!!
                            dest_node = s_mapRFIDtoNode[TrafficManager::override_dest[i].rfid];
                            //Still add this AGV to free list
                            free_agv.push_back(i);
                            free_agv_list << agvid;
                        }
                        else //Bin is now associated with the AGV
                        {
                            if (TrafficManager::override_dest[i].rfid != "")
                            {
                                TrafficManager::override_dest[i].rfid = ""; // clear any over ride destination. Bin takes precedence
                                charging_stations[TrafficManager::override_dest[i].charging_station_index].free = true;
                                TrafficManager::override_dest[i].charging_station_index = -1;
                            }
                            //Decide whether bin is to be picked up, taken to pick station/replenish station or put back
                            bin_status = agvTable.record(i).value("AgvBinStatus").toString();
                            if (bin_status == "EMPTY")//go to bin location
                            {
                                dest_rfid = agvTable.record(i).value("BinLocationId").toString();
                                //qDebug() <<"Target location is dest rfid" << dest_rfid;
                                dest_node = s_mapRFIDtoNode[dest_rfid];
                            }
                            else
                            {
                                /*//Bin is loaded first check if items to pick are non-zero
                                items_to_pick = agvTable.record(i).value("ItemCountToPick").toInt();
                                items_to_replenish = agvTable.record(i).value("ItemCountToPut").toInt();
                                if (items_to_pick)
                                {
                                    dest_rfid = agvTable.record(i).value("BinTargetLocationId").toString();
                                    dest_node = s_mapRFIDtoNode[dest_rfid];
                                }
                                else if (items_to_replenish)
                                {
                                    dest_rfid = agvTable.record(i).value("BinTargetLocationId").toString();
                                    dest_node = s_mapRFIDtoNode[dest_rfid];
                                }
                                else
                                {*///Put loaded bin back both pick and replenish actions are completed or not needed.
                                    dest_rfid = agvTable.record(i).value("BinTargetLocationId").toString();
                                    //qDebug() <<"Target location is dest rfid" << dest_rfid;
                                    dest_node = s_mapRFIDtoNode[dest_rfid];
                                //}
                            }//destination is set
                            if(dest_rfid.length() < 10)
                            {
                                qDebug() <<"dest rfid in override" << dest_rfid;
                                continue;
                            }
                        }

                        //Set source for AGV path planning
                        if (goal_RFID == "")
                        {
                            source_rfid = last_node_rfid;
                        }
                        else if (last_node_rfid == goal_RFID)//pause or reached new node case
                        {
                            if (agv_goal_behavior == "turn")
                            {
                                source_rfid = last_node_rfid;
                            }
                            else
                            {
                                int to_find = map_behave_stringtoint[agv_goal_behavior];
                                int index = 0;
                                for (index = 0; index < number_of_nodes; index++)
                                {
                                    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl << i << " Behave graph = " << behave_graph[goal_RFID_node][index] << endl;
                                    if ((int)behave_graph[goal_RFID_node][index] == to_find)
                                        break;
                                }
                                //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl << "Location = " << index << endl;
                                source_rfid = s_mapNodetoRFID[Nodenum(index)];
                            }
                        }
                        // else if (dist_from_last_rfid > TrafficManager::dist_between_rfid[last_node][goal_RFID_node] + 0.25 && TrafficManager::visibility_map[last_node][goal_RFID_node] == 1)
                        // {
                        //      //RFID has been skipped - change source to next one
                        //      int to_find = map_behave_stringtoint[agv_current_behavior];
                        //      int index = 0;
                        //      for (; index < number_of_nodes; index++)
                        //      {
                        //          if((int)behave_graph[goal_RFID_node][index] == to_find)
                        //              break;
                        //       }
                        //      if (index == number_of_nodes)
                        //      {
                        //          //Re-search the behavior
                        //      }
                        // }
                        else
                        {
                            source_rfid = goal_RFID;
                        }

                        source_node = s_mapRFIDtoNode[source_rfid];//source node is set
                        if (last_node != source_node)
                        {
                            agv_current_orientation[i] = TrafficManager::orientation_at_edge_end[last_node][source_node];
                            //qDebug() <<"AGVOrientation is updated here as " << agv_current_orientation[i];
                            //qDebug() <<"Last node is " << last_node << "source node" << source_node;
                        }
#if LIVE_MODE == 1
                        else
                        {
                            agv_current_orientation[i] = agvTable.record(i).value("AgvGlobalOrientation").toInt();
                        }
#endif
#if DEBUG_FULL_PLAN
                        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Source node for " << agvid.toStdString() << "=" << source_node << " RFID = " << source_rfid.toStdString() << " Dest node = " << dest_node << endl;
                        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Last RFID = " << last_node_rfid.toStdString() << ", " << " Goal = " << goal_RFID.toStdString() << " Next to next = " << agv_goal_behavior.toStdString() << endl << endl;
#endif
                        double dist_to_goal_rfid = 0.0;
#if SAME_SPEED_SIMULATION == 0
                        dist_to_goal_rfid = TrafficManager::dist_between_rfid[last_node][source_node] - dist_from_last_rfid;
#endif
                        int success_fail = add_path_to_plan(source_node, dest_node, i, extra_edge_to_add[i], \
                                                            bin_id_list, dist_to_goal_rfid, last_node, agv_current_orientation[i]);
#if LIVE_MODE == 0
                        if (last_node == source_node && agv_goal_behavior == "turn")
                        {
                            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Turning the AGV";
                            sleep(3);
                            agv_current_orientation[i] = TrafficManager::orientation_at_edge_start[source_node][goal_node];
                            TrafficManager::turnaround_at_src[i] = 0;//turn done
                        }
#endif

#if LIFTER_LOGIC
                        /* override dest node is sent in case diverter has not acked */
                        int override_dest_node = lift_request_func(i, last_node_rfid);
                        if (override_dest_node != -1)
                        {
                            dest_node = override_dest_node;
                            dest_rfid = s_mapNodetoRFID[Nodenum(dest_node)];
                            success_fail = add_path_to_plan(source_node, dest_node, i, extra_edge_to_add[i], \
                                                                                        bin_id_list, dist_to_goal_rfid, last_node, agv_current_orientation[i]);
                            qDebug() << "$$$$$$ Over ride Dest Node : " <<  dest_node;
                        }
#endif

#if DIVERTER_LOGIC
                        /* override dest node is sent in case diverter has not acked */
                        int override_dest_node = diverter_request_func(i, last_node_rfid);
                        if (override_dest_node != -1)
                        {
                            dest_node = override_dest_node;
                            dest_rfid = s_mapNodetoRFID[Nodenum(dest_node)];
                            success_fail = add_path_to_plan(source_node, dest_node, i, extra_edge_to_add[i], \
                                                                                        bin_id_list, dist_to_goal_rfid, last_node, agv_current_orientation[i]);

                        }
#endif
                        //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Success_fail = " << success_fail << endl;
                        if (success_fail == 0 && source_node != dest_node)
                        {
                            no_progress_to_goal_counter[i]++;
                            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Standstill for AGV " << i << " increased = " << no_progress_to_goal_counter[i] << endl;
                            if (no_progress_to_goal_counter[i] > 100 && bin_status == "EMPTY")
                            {
                                //reliquinsh bin assigned to this AGV
                                DbRemoveAgvBin(TrafficManager::db, agvid);
#if DEBUG_MRPP
                                BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl << endl << endl << "!!!!Removed bin associated with AGV no. !!!! " << agvid.toStdString() << endl << endl << endl;
#endif
                            }
#if DEBUG_MRPP
                            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Failure in planning path for agent " << i << endl;
#endif
                        }
                        else
                        {
                            no_progress_to_goal_counter[i] = 0;
                        }

                        //next RFID and behavior update frm current approved_time_plan & path_list
                        goal_RFID =  agvTable.record(i).value("AgvGoal").toString();
                        if (goal_RFID == "")
                        {
                            //new goal node needs to be supplied along with next to next node
                            if (path_list[i].size() > 1)
                            {
                                goal_node = TrafficManager::path_list[i][1];
                                goal_RFID = s_mapNodetoRFID[Nodenum(goal_node)];
                            }
                            else
                            {
                                goal_node = TrafficManager::path_list[i][0];
                                goal_RFID = s_mapNodetoRFID[Nodenum(goal_node)];
                            }
                            if (TrafficManager::path_list[i].size() > 2)
                            {
                                next_next_RFID_node = TrafficManager::path_list[i][2];
                                next_next_RFID = s_mapNodetoRFID[Nodenum(next_next_RFID_node)];
                                agv_goal_behaviour = map_behave_enumtostring[agent_behaviour(behave_graph[goal_node][next_next_RFID_node])];
                            }
                            else
                            {
                                agv_goal_behaviour = "pause";
                            }
                            currbehave = map_behave_enumtostring[agent_behaviour(behave_graph[source_node][goal_node])];
                        }
                        else
                        {
                            goal_node = TrafficManager::path_list[i][0];
                            goal_RFID = s_mapNodetoRFID[Nodenum(goal_node)];
                            if (TrafficManager::path_list[i].size() > 1)
                            {
                                next_next_RFID_node = TrafficManager::path_list[i][1];
                                next_next_RFID = s_mapNodetoRFID[Nodenum(next_next_RFID_node)];
                                agv_goal_behaviour = map_behave_enumtostring[agent_behaviour(behave_graph[goal_node][next_next_RFID_node])];
                            }
                            else
                            {
                                agv_goal_behaviour = "pause";
                            }
                            currbehave = map_behave_enumtostring[agent_behaviour(behave_graph[last_node][goal_node])];
                        }

#if DEBUG_MRPP_BEHAVIOURS
                        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Curr behaviour = " << currbehave.toStdString() << endl;
                        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "For " << agvid.toStdString() << " Updating AGV goal " << goal_RFID.toStdString() << " Goal behavior " << agv_goal_behaviour.toStdString() << endl;
#endif
                        if (TrafficManager::turnaround_at_src[i] == 1)
                        {
                            agv_goal_behaviour = "turn";
                            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "AGV " << agvid.toStdString() << " does turnaround" << endl;
                        }
#if LIVE_MODE == 1
                        if (last_node == source_node && TrafficManager::turnaround_at_src[i] == 1)
                        {
                            agv_goal_behaviour = "pause";
                            currbehave = "turn";
                        }
//                        if (last_node == source_node && agv_goal_behavior == "turn")
//                        {
//                            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Turning the AGV";
//                            //sleep(1);
//                            agv_current_orientation[i] = TrafficManager::orientation_at_edge_start[source_node][goal_node];
//                            TrafficManager::turnaround_at_src[i] = 0;//turn done
//                        }
#endif
                        DbUpdateAgvGoal(TrafficManager::db, agvid, goal_RFID, currbehave, agv_goal_behaviour, QString::number(TrafficManager::distance_map[last_node][goal_node]), QString::number(agv_current_orientation[i]));

                    }
                    else//AGV is free with no bin associated - add it to free AGV list
                    {
                        free_agv.push_back(i);
                        free_agv_list << agvid;
                    }
                }
            }

#if DEBUG_FULL_PLAN
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl << endl;
            //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << TrafficManager::orientation_at_edge_end[34][35];
            show_approved_plan();
            show_approved_timeplan();
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << endl << endl;
#endif

            //If AGV had bin or was busy this has been handled
            //Now if more than one AGV is free
            //Check for active unassigned bins and assign to free AGVs.
            if (free_agv_list.size())
            {

                //Query for bins to be returned to rack
                QSqlQueryModel scheduledInactiveBinsTable;
                DbSelectScheduledInactiveBins(db, &scheduledInactiveBinsTable);

                QSqlQueryModel bin_list;
                DbSelectScheduledBins(TrafficManager::db, &bin_list);

                if ((bin_list.rowCount() == 0) && (scheduledInactiveBinsTable.rowCount() == 0))
                {
                    //BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "MRPP No bins to schedule" << endl;
                    continue;
                }
                vector<int> bins_nodes, src_nodes;
                QVariantList bin_ids_tosched;
                QString bin_id, bin_location_RFID, src_RFID;
                int bin_node, src_node;

#if DEBUG_MRPP_BINS
                BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Bins to be scheduled " << endl;
#endif

                for(int i = 0; i < scheduledInactiveBinsTable.rowCount(); i++)
                {
                    bin_id = scheduledInactiveBinsTable.record(i).value("BinId").toString();
                    QSqlQueryModel binTable;
                    DbSelectBinLocation(db, &binTable, bin_id);
                    bin_location_RFID = binTable.record(0).value("LocationId").toString();
                    bin_node = s_mapRFIDtoNode[bin_location_RFID];
                    bins_nodes.push_back(bin_node);
                    bin_ids_tosched << bin_id;
                }

                for (int i = 0; i < bin_list.rowCount();i++)
                {
                    bin_id = bin_list.record(i).value("BinId").toString();
#if DEBUG_MRPP_BINS
                    BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << bin_id.toStdString() << " ";
#endif
                    QSqlQueryModel binTable;
                    DbSelectBinLocation(TrafficManager::db, &binTable, bin_id);
                    bin_location_RFID = binTable.record(0).value("LocationId").toString();
                    QSqlQueryModel binRfidTypeTable;
                    DbSelectRfidtype(db, &binRfidTypeTable, bin_location_RFID);
                    QString bin_location_RFID_type = binRfidTypeTable.record(0).value("RfLocationType").toString();
                    if((bin_location_RFID_type == "RACK") || (bin_location_RFID_type == "PICKSTATIONRETRIEVEPOINT"))
                    {
                        bin_node = s_mapRFIDtoNode[bin_location_RFID];
                        bins_nodes.push_back(bin_node);
                        bin_ids_tosched << bin_id;
                    }
                }

                for (int i = 0; i < free_agv_list.size();i++)
                {
                    src_RFID = agvTable.record(free_agv[i]).value("AgvLastRfid").toString();
                    src_node = s_mapRFIDtoNode[src_RFID];
                    src_nodes.push_back(src_node);
#if DEBUG_MRPP_BINS
                    BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "Source for free AGV " << free_agv[i] << " is " << src_node << endl;
#endif
                }
                //calculate costs of free AGVs to bins and assign
                calc_costs_to_binlocs_andassign(src_nodes, bins_nodes, free_agv_list, bin_ids_tosched, free_agv, bin_id_list);
            }
            else
            {
#if DEBUG_MRPP_BINS
                BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", normal) << "No Agvs are free to pick bins " << endl;
#endif
            }
        }
    }
}

#ifdef MAIN_USE
int main(int argc, char *argv[])
{
    class TrafficManager obj_main;

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    obj_main.initialize_static_map_and_traffic();//map, number_nodes, costs are initialized, approved_plan

    //thread which updates the approved_time_plan every 1.50 seconds - interacts with UI and updates approved_time_plan
    //pthread_create(&timeplan_update_thread, NULL, &timeplan_update_simulation, NULL);
    //pthread_t timeplan_update_thread;
    int ret = pthread_create(&timeplan_update_thread, NULL,TrafficManager::callthread_func,&obj_main);
    //pthread_join(timeplan_update_thread,NULL);
}

#else
/*! \brief gettasks_and_start_traffic_manager_thread
 *         Starting point of Traffic Manager
 *
 *  Creates main traffic manager thread and initializes static map from path passed by UI.
 *  Returns 0 on success, -1 on failure.
 *
 */
int TrafficManager::gettasks_and_start_traffic_manager_thread(QString map_files_path)
{
    int ret = initialize_static_map_and_traffic(map_files_path);//map, number_nodes, costs are initialized, approved_plan
    //cout << "Map file path = " << map_files_path.toStdString() << endl;
    if (ret == -1)
    {
        BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", critical) << "Error in initializing Traffic Manager map and traffic ";
        return -1;
    }
    else
    {
        /*thread which updates the approved_time_plan every 100 milliseconds - interacts with UI and updates approved_time_plan*/
        int ret = pthread_create(&timeplan_update_thread, NULL,TrafficManager::callthread_func,this);
        //int ret1 = pthread_create(&simulate_thread, NULL, TrafficManager::call_simulate_goal, this);
        if (ret == -1)
        {
            BOOST_LOG_CHANNEL_SEV(lg, "trafficmanager", critical) << "Error in starting MRPP thread";
            return -1;
        }
    }
    return 0;
}

#endif
