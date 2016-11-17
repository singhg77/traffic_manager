#ifndef TRAFFICMANAGER_H
#define TRAFFICMANAGER_H

#include <vector>
#include <QQuickItem>
#include "db.h"
#include <sstream>
#include <iomanip>

#define DEBUG_MRPP 0
#define DEBUG_COLLISION 0
#define DEBUG_MRPP_LADDER 0
#define DEBUG_MRPP_BINS 0
#define DEBUG_MRPP_BEHAVIOURS 0
#define DEBUG_FULL_PLAN 0
#define DEBUG_DJIK 0

using namespace std;
typedef vector < vector<int> > doubledim_vec;
struct collision_details
{
	int coll_yesno;
	int node_edge_no;
};

struct path_behaviour_and_cost
{
    vector<int> path;
    double cost;
    vector<int> behaviour;
};

struct agv_diverter_request
{
    QVariantList diverter_rfid;
    int agv_request_status;
    QString ladder_config_accepted;
};

struct charging_station
{
    int node_no;
    QString rfid;
    bool free;
};

struct override_destination
{
    QString rfid;
    int charging_station_index;
};

struct speed_agv
{
    double straight_speed_agv;//cmp
    double curve_speed_agv;//cmps
    double rack_speed_agv;//cmps
    double default_speed;//cmps
};

void timeplan_update_simulation();

class TrafficManager: public QObject
{
	private:
		Q_OBJECT
        static doubledim_vec approved_time_plan;//double dim array of AGV positions in forecasted time
		static vector<int> agent_status; //double dim array of AGV free/not free in forecasted time
		static doubledim_vec path_list; //current path list of every AGV - to be communicated with UI
        static vector<int> turnaround_at_src;
        static vector < vector< double > > distance_map;
		static doubledim_vec visibility_map;
		static doubledim_vec link_no;
		static doubledim_vec next_highlevel_dests; //next destination (node no) of every AGV - will be read at completion of tasks

        static doubledim_vec orientation_at_edge_start;
        static doubledim_vec orientation_at_edge_end;

		static int number_of_nodes;
        vector< vector< double > > dist_between_rfid;
		static int pickup_node;
		static int replenish_node;
        static bool status_loop_started;
        static doubledim_vec behave_graph;
        static doubledim_vec adjacent_edges;
        static doubledim_vec adjacent_nodes;
        static vector<struct agv_diverter_request> agv_request_diverter;
        static vector<struct charging_station> charging_stations;
        int no_of_charging_stations;
        static vector<struct override_destination> override_dest;
        vector < vector <double> > agv_time_cost_map;
        vector < QString > agvId;
        QSqlDatabase db;

		Q_PROPERTY (QString uipathadded   READ uipathadded      WRITE setuipathadded
				NOTIFY uipathaddedChanged)

        QString m_uipathadded;

		int m_agvUIid;

	public:
		TrafficManager()
		{
		}

        //Q_INVOKABLE vector<int> path_totimeplan_realtime(vector<int> path_toconvert, double dist_to_next_rfid, vector<double> speeds); //using exact distance
        Q_INVOKABLE vector<int> path_totimeplan_realtime(vector<int> path_toconvert, double dist_to_next_rfid, int extra_edge_to_add, int last_node);

        Q_INVOKABLE vector<int> path_totimeplan_simulation(vector<int> path_toconvert, int extra_edge_to_add);// it is an approximation

        Q_INVOKABLE struct collision_details check_collision(vector<int> time_plan_part, \
                vector< vector<int> > approved_time_plan, int agent_to_add, QVariantList bin_id_list);//Done

        Q_INVOKABLE path_behaviour_and_cost djik_algo(int source, int destination, int no_of_nodes);//Done

        Q_INVOKABLE path_behaviour_and_cost space_time_djik_simulation(int source, int dest, vector< vector<int> > approved_time_plan, \
                                                                int number_of_nodes, int agent_to_add, \
                                                                int extra_edge_to_add, QVariantList bin_id_list, int starting_orientation, int last_node);


        path_behaviour_and_cost space_time_djik_realtime(int source, int dest, vector < vector <int> > approved_time_plan, int number_of_nodes, \
                                                             int agent_to_add, int extra_edge_to_add, double distance_to_goal_rfid, \
                                                             QVariantList bin_id_list, int starting_orientation, int last_node);

        Q_INVOKABLE int initialize_static_map_and_traffic(QString map_files_path);//Done

        Q_INVOKABLE int add_path_to_plan(int source, int next_dest, int agent_id, int extra_edge_to_add, QVariantList bin_id_list, double dist_to_goal_rfid, int last_node, int agv_current_orientation);

        Q_INVOKABLE void agv_time_cost_map_preparation(QString agvId);

        Q_INVOKABLE void mrpp_main_thread();

        Q_INVOKABLE void simulate_goal();

        Q_INVOKABLE static void* callthread_func(void *arg) {
            void *send_back = NULL;
            ((TrafficManager*)arg)->mrpp_main_thread();
            return send_back;
        }

        Q_INVOKABLE static void* call_simulate_goal(void *arg) {
            void *send_back = NULL;
            ((TrafficManager*)arg)->simulate_goal();
            return send_back;
        }

        Q_INVOKABLE int gettasks_and_start_traffic_manager_thread(QString map_files_path);

		Q_INVOKABLE void show_approved_timeplan(void);
		
		Q_INVOKABLE void show_approved_plan(void);

        Q_INVOKABLE void calc_costs_to_binlocs_andassign(vector<int> srcs, vector<int> dests, QVariantList free_agv_list, QVariantList bin_ids_tosched, vector<int> free_agv, QVariantList bin_id_list);

        Q_INVOKABLE int overlap_check(int agent_pos, int agent_to_add_pos);

        Q_INVOKABLE int diverter_request_func(int agvindex, QString last_node_rfid);

        Q_INVOKABLE int lift_request_func(int agvindex, QString last_node_rfid);

        QString uipathadded() const
		{
			return m_uipathadded;
		}

        void pause_traffic_manager()
        {
            TrafficManager::status_loop_started = false;
        }

        void unpause_traffic_manager()
        {
            TrafficManager::status_loop_started = true;
        }

        bool read_visibility_map(QString map_files_path);

        bool read_behaviour_map(QString map_files_path);

		public slots:
			/// Setter functions for the variables accessible through QML.
			void setuipathadded(QString arg)
			{
				//        if (m_UIpathadded == arg)
				//            return;
				fprintf(stderr,"Send path = %s",arg.toStdString().c_str());
				m_uipathadded = arg;
				emit uipathaddedChanged(arg);
			}

signals:
		void uipathaddedChanged(QString arg);
};

#endif
