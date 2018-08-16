#ifndef MAIN_H
#define MAIN_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the quadrotor positioning actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the eye-bot proximity sensor */
#include <argos3/plugins/robots/eye-bot/control_interface/ci_eyebot_proximity_sensor.h>
/* Definition of the perspective camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>
/* Definition of the eye-bot light sensor */
#include <argos3/plugins/robots/eye-bot/control_interface/ci_eyebot_light_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definitions for the argos space */
#include <argos3/core/simulator/space/space.h>
/* Include the kalman filter algorithm definitions */
#include <filters/kalman/kalman.h>
/* Definition of the argos entities */
#include <argos3/plugins/simulator/entities/light_entity.h>
/* Include the pso swarm algorithm definitions */
#include <algorithms/pso/swarm.h>
/* Definitions for the eigen library */
#include <Eigen/Dense>
// basic file operations
#include <fstream>
#include <sstream>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEyeBotMain : public CCI_Controller {

public:
    /* Class constructor. */
    CEyeBotMain();

    /* Class destructor. */
    virtual ~CEyeBotMain() {}

    /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><plant_targets>
    * section.
    */

    virtual void Init(TConfigurationNode& t_node);
    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
    virtual void Reset();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    /*
    * Controller state data
    */
    struct SStateData {
        /* Y-axis reach to the wall/target space */
        double Reach;
        /* Boolean to determine if drone is leader */
        bool IsLeader;
        /* Current robot state */
        enum EState {
            STATE_START = 0,
            STATE_MOVE,
            STATE_EXECUTE_TASK,
            STATE_REST,
            STATE_LAND
        } State;

        /* Current robot task */
        enum ETask {
            TASK_EVALUATE = 0,
            TASK_WATER,
            TASK_TREATMENT,
            TASK_NOURISH,
            TASK_NULL,
            TASK_INVALID
        } TaskState;

        /* Current robot color */
        CColor TaskColor;
        /* Attitude height above target to hold task execution */
        double attitude;
        /* Average plane distance to wall to move along the path */
        double global_reach;
        /* Tolerance threshold for the distance to a target point */
        double proximity_tolerance;
        /* The minimum number of steps in holding mode before the eyebot can advance waypoints */
        double minimum_hold_time;
        /* The minimum number of steps to wait before replanning unordered waypoints */
        double minimum_rest_time;
        /* Initial probability to switch from resting to moving */
        double InitialRestToMoveProb;
        /* Current probability to switch from resting to exploring */
        double RestToMoveProb;
        /* The increase of RestToMoveProb due to the social rule */
        double SocialRuleRestToMoveDeltaProb;
        /* Initial probability to switch from moving to landing */
        double InitialRestToLandProb;
        /* Current probability to switch from moving to landing */
        double RestToLandProb;
        /* The increase of RestToLandProb due to the social rule */
        double SocialRuleRestToLandDeltaProb;
        /* Current robot waypoint location index both in local and global maps */
        size_t LocalIndex;
        /* Time that the drone will hold at target while it performs task */
        size_t HoldTime;
        /* Time that the drone will remain in rest mode */
        size_t RestTime;

        void Init(TConfigurationNode& t_node);
        void Reset();
    };

    inline void UpdateWaypoint() {
        if(m_sStateData.HoldTime > m_sStateData.minimum_hold_time) {
            m_sStateData.LocalIndex++;
            m_sStateData.HoldTime = 0;
        } else {
            m_sStateData.HoldTime++;
        }
    }

    inline void ActOnTarget(std::string action) {
        SStateData::ETask target_task;
        CColor target_color;

        for(size_t t_id = 0; t_id < m_pTargetMap.size() ; t_id++) {
            if(action == std::get<0>(m_pTargetMap[t_id])) {
                target_task = std::get<1>(m_pTargetMap[t_id]);
                target_color = std::get<2>(m_pTargetMap[t_id]);

                if(m_cNearestTarget->GetColor() == target_color && m_sStateData.TaskState == target_task) {
                    if(m_sRandGen.taskcompleted.get()) {
                        LOG << "completing " << action << " task!";
                        m_cNearestTarget->SetColor(CColor::GREEN);
                        std::get<1>(GlobalMap[m_sStateData.LocalIndex]) = SStateData::TASK_NULL;
                        IncreaseLandingProb();
                    }  else {
                        LOG << action << " task interrupted/not completed!";
                    }
                } else if(m_cNearestTarget->GetColor() == CColor::GREEN) {
                    LOG << "found healthy (green) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
                    std::get<1>(GlobalMap[GetGlobalIndex()]) = SStateData::TASK_NULL;
                    SendTask(SStateData::TASK_NULL);
                }
            }
        }
        SendTask(SStateData::TASK_NULL);
        UpdateWaypoint();
    }

    inline size_t GetGlobalIndex() {
        size_t g_id;
        // Search for index to global map waypoint
        for(auto& gwp : GlobalMap) {
            if(std::get<0>(gwp.second) == std::get<0>(LocalMap[m_sStateData.LocalIndex])) {
                g_id = gwp.first;
            }
        }
        return (g_id);
    }

    inline std::vector<double> GetWaypoint() {
        return (std::get<0>(LocalMap[m_sStateData.LocalIndex]));
    }

    inline CVector3 GetPosition() {
        return (m_sKalmanFilter.state);
    }

    inline void IncreaseLandingProb() {
        // Increase probability that robot will go into land state.
        m_sStateData.RestToLandProb += m_sStateData.SocialRuleRestToLandDeltaProb;
        // Truncate RestToLand probability value.
        m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRandGen.resttoland.max()), m_sRandGen.resttoland.min());
        // Decrease probability that robot will go into move state.
        m_sStateData.RestToMoveProb -= m_sStateData.SocialRuleRestToMoveDeltaProb;
        // Truncate RestToMove probability value.
        m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRandGen.resttomove.max()), m_sRandGen.resttomove.min());
    }

    inline void IncreaseMovingProb() {
        // Increase probability that robot will go into move state.
        m_sStateData.RestToMoveProb += m_sStateData.SocialRuleRestToMoveDeltaProb;
        // Truncate RestToMove probability value.
        m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRandGen.resttomove.max()), m_sRandGen.resttomove.min());
        // Decrease probability that robot will go into land state.
        m_sStateData.RestToLandProb -= m_sStateData.SocialRuleRestToLandDeltaProb;
        // Truncate RestToLand probability value.
        m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRandGen.resttoland.max()), m_sRandGen.resttoland.min());
    }

    inline void SendTask(SStateData::ETask task) {

        if(m_sStateData.HoldTime == 1 && task != SStateData::TASK_INVALID && strcmp(m_sExperimentParams.name, "lawn")) {
            CByteArray cBuf(10);
            cBuf[0] = (UInt8)task                 & 0xff;
            cBuf[1] = (UInt8)GetGlobalIndex()     & 0xff;

            m_pcRABA->SetData(cBuf);
            LOG << "sent";
        } else {
            LOG << "cancelled";
        }
        LOG << std::endl;
    }

private:

    /*
    * Lands the robot.
    */
    void Land();

    /*
    * Move robot to next waypoint.
    */
    void Move();

    /*
    * Rest time before replanning waypoints.
    */
    void Rest();

    /*
    * Perform the requisite task on the target plant.
    */
    void ExecuteTask();

    /*
    * Based on the assigned tag perform varied tasks.
    * White - reassign tag to plant
    * Green - leave plant alone
    * Yellow - apply medication
    * Red - water the plant
    */
    void EvaluateFunction();
    void WaterFunction();
    void NourishFunction();
    void TreatmentFunction();

private:

    /*
    * Perform basic kalman filtering on quadcopter
    * position measurements.
    */
    void UpdatePosition(CVector3 x0 = CVector3(0.,0.,0.));

    /*
    * Update the target light entity to the
    * nearest light led.
    */
    void UpdateNearestTarget();

    /*
    * Distribute tasks between available eye-bots
    * in the arena. The initializer must be run before
    * we can generate waypoints.
    */
    void InitializeDrones();

    /*
    * Social rule listener
    */
    void ListenToNeighbours();

   /*
    * initialize and waypoint targets in the arena: this can be done naively
    * with the passed argos parameters or with the help
    * of the camera sensor.
    */
    void InitializeGlobalMap(bool verbose = false);
    /*
    * Generate optimal path for waypoints listed in GlobalMap.
    */
    void UpdateLocalMap(bool verbose = false);

    /*
    * Record and update trials when complete.
    */
    void RecordTrial();

    /*
    * The swarm params.
    */
    struct SSwarmParams {
        int ants;
        int particles;
        double self_trust;
        double past_trust;
        double global_trust;

        void Init(TConfigurationNode& t_node);
    };

    /*
    * Simple Kalman filter struct
    */
    struct SKF {
        int n = 3; // Number of states
        int m = 3; // Number of measurements
        double dt = 10/60; // Time step

        Eigen::MatrixXd A = Eigen::MatrixXd(n, n); // System dynamics matrix
        Eigen::MatrixXd C = Eigen::MatrixXd(m, n); // Output matrix
        Eigen::MatrixXd Q = Eigen::MatrixXd(n, n); // Process noise covariance
        Eigen::MatrixXd R = Eigen::MatrixXd(m, m); // Measurement noise covariance
        Eigen::MatrixXd P = Eigen::MatrixXd(n, n); // Estimate error covariance

        // Construct the state vector
        CVector3 state;
        SKF();
    };

    /*
    * Noise sources
    */
    struct SGaussDist {
        /* Define random generator with Gaussian distribution */
        std::default_random_engine* gen;
        std::normal_distribution<double>* nd;

        void Init(double& mean, double& stddev, int& gen_seed);
        double min() {
            return nd->min();
        };
        double max() {
            return nd->max();
        };
        double get() {
            return (*nd)(*gen);
        };
    };

    struct SUniformIntDist {
        /* Define discete random generator with Uniform distribution */
        std::default_random_engine* gen;
        std::uniform_int_distribution<int>* uid;

        void Init(int min, int max, int& gen_seed);
        int min() {
            return uid->min();
        };
        int max() {
            return uid->max();
        };
        int get() {
            return (*uid)(*gen);
        };
    };

    struct SUniformRealDist {
        /* Define continuous random generator with Uniform distribution */
        std::default_random_engine* gen;
        std::uniform_real_distribution<double>* udd;

        void Init(double min, double max, int& gen_seed);
        double min() {
            return udd->min();
        };
        double max() {
            return udd->max();
        };
        double get() {
            return (*udd)(*gen);
        };
    };

    /*
    * Random generators
    */
    struct SRandomGen {
        double mapping_mean,mapping_stddev,rtm_min,rtm_max,rtl_min,rtl_max,task_completed_min,task_completed_max,target_shuffle_min,target_shuffle_max;
        int mapping_seed,rtm_seed,rtl_seed,task_completed_seed,target_shuffle_seed,aco_seed;

        SGaussDist mapping;
        SUniformIntDist targetshuffle;
        SUniformIntDist taskcompleted;
        SUniformRealDist resttomove;
        SUniformRealDist resttoland;

        void Init(TConfigurationNode& t_node);
        void Set(size_t target_states_size);
    };

    /*
    * Experiment parameters
    */
    struct SExperimentParams {
        bool naive_mapping;
        char name[4];
        size_t trials;
        double target;

        void Init(TConfigurationNode& t_node);
    };

    /*
    * Lawn parameters
    */
    struct SLawnParams {
        /* Horizontal and Vertical step values */
        /* Distance to move vertically along the lawn length at */
        double vstep;
        /* Distance to move horizontally along the lawn length at */
        double hstep;

        void Init(TConfigurationNode& t_node);
    };

private:

    /* Pointer to the quadrotor position actuator */
    CCI_QuadRotorPositionActuator* m_pcPosAct;
    /* Pointer to the positioning sensor */
    CCI_PositioningSensor* m_pcPosSens;
    /* Pointer to the eye-bot proximity sensor */
    CCI_EyeBotProximitySensor* m_pcProximity;
    /* Pointer to the perspective camera sensor */
    CCI_ColoredBlobPerspectiveCameraSensor* m_pcCamera;
    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator*  m_pcRABA;
    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABS;
    /* Contains the message received from the foot-bot */
    const CCI_RangeAndBearingSensor::SPacket* m_pEBMsg;

    /* Current target position */
    CVector3 m_cTargetPos;
    /* Perspective camera readings variable */
    CCI_ColoredBlobPerspectiveCameraSensor::SReadings m_cSReadings;
    CVector3 HomePos;

    /*
     * References to simulated space variables.
     */
    CSpace* m_pcSpace;
    KalmanFilter* kf;
    CLightEntity* m_cNearestTarget;
    // Pointer to task function.
    void (CEyeBotMain::*TaskFunction)();

    // The controller state information
    SStateData m_sStateData;
    SExperimentParams m_sExperimentParams;
    SSwarmParams m_sSwarmParams;
    SLawnParams m_sLawnParams;
    SRandomGen m_sRandGen;
    SKF m_sKalmanFilter;

    /* Swarm solution variables */
    std::vector<int> tour;
    long int tour_length;

    /* Flag to indicate when the drones have been initialized */
    bool drones_initialized;

    /* Eyebot task mapping:
    * WHITE     - Undecided/Unknown
    * GREEN     - Healthy
    * YELLOW    - Diseased
    * BROWN     - Dry
    * RED       - Wilting
    */
    std::map< size_t, std::tuple<std::string, SStateData::ETask, CColor, double> > m_pTargetMap = {
        {0, std::make_tuple("evaluate", SStateData::TASK_EVALUATE, CColor::WHITE, 0.6)},
        {1, std::make_tuple("water", SStateData::TASK_WATER, CColor::BROWN, 0.0)},
        {2, std::make_tuple("nourish", SStateData::TASK_NOURISH, CColor::YELLOW, -0.6)},
        {3, std::make_tuple("treatment", SStateData::TASK_TREATMENT, CColor::RED, -1.2)},
        {4, std::make_tuple("null", SStateData::TASK_NULL, CColor::GREEN, 0.0)}
    };

    std::map<std::string, SStateData::ETask> m_mTaskedEyeBots;

    /* Current robots' global and local waypoint maps */
    std::map<size_t, std::tuple< std::vector<double>, SStateData::ETask, CColor >> GlobalMap, LocalMap;

    // File to record simulation data to.
    std::string m_sFile;
    bool fileCreated;
    size_t fileCounter = 0;
    // Start from 1 for sound logic.
    size_t trialCounter = 1;
};

#endif