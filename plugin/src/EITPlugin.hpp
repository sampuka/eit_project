#ifndef EITPLUGIN_HPP
#define EITPLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/models/WorkCell.hpp>

// Trajectory planning
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/ProximityData.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

// RobWorkStudio includes
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

// Qt
#include "ui_EITPlugin.h"


// RTDE
//#include <rwhw/universalrobots_rtde/URRTDE.hpp>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

// Standard includes
#include <exception>
#include <memory>
#include <thread>

struct SubPath
{
    SubPath(std::vector<std::vector<double>> _path, rw::trajectory::InterpolatorTrajectory<rw::math::Q> _traj, bool _grip)
        : path(_path), traj(_traj), grip(_grip)
    {}
    SubPath() = delete;

    const std::vector<std::vector<double>> path;
    const rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj;
    const bool grip;
};

class EITPlugin: public rws::RobWorkStudioPlugin, private Ui::EITPlugin
{
    Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
    //QThread connectThread;

    public:
        EITPlugin();
        virtual ~EITPlugin();

        virtual void open(rw::models::WorkCell* workcell);
        virtual void close();
        virtual void initialize();

    private slots:
        void stateChangedListener(const rw::kinematics::State& state);

        void button_connect_disconnect();
        void button_freemode();
        void button_home();
        void apply_force(double force);
        void button_start();
        void sync_pressed(bool);
        void create_trajectory(rw::math::Q from, rw::math::Q to, double extend, double vel = 1.05);

    private:
        void move_ur(rw::math::Q from, rw::math::Q to);

        bool should_shutdown = false; // Set to true -> threads stop

        // Control loop
        bool running = false; // Whether or not robot moves
        void control_loop();
        std::thread control_loop_thread;
        std::vector<std::pair<unsigned int, rw::math::Q>> trajectory; // unsigned int is amount of milliseconds until next position
        unsigned int trajectory_index = 0; // Which index of the trajectory are we on, or moving towards
        rw::trajectory::InterpolatorTrajectory<rw::math::Q> trash;
        std::vector<std::vector<double>> path;
        std::vector<SubPath> whole_path;
        unsigned int path_section = 0;

        // Planning
        rw::math::Q home_Q;
        rw::math::Q pick_approach_Q;
        rw::math::Q pick_Q;
        const unsigned int place_position_count = 3; // Must be at least 2 for now
        const double x_lim1 = 0.35;
        const double x_lim2 = -0.35;
        std::vector<rw::math::Q> place_approach_Qs;
        std::vector<rw::math::Q> place_Qs;

        void create_whole_path();
        std::thread create_whole_path_thread;

        // RobWork
        rw::models::WorkCell::Ptr rws_wc;
        rw::kinematics::State rws_state;
        rw::models::SerialDevice::Ptr UR_robot;
        rw::models::TreeDevice::Ptr gripper;
        rw::kinematics::Frame::Ptr base_frame;
        rw::kinematics::MovableFrame::Ptr rebar_frame;

        // UR Robot
        const std::string ur_ip = "10.10.1.100";
        // std::unique_ptr<rwhw::URRTDE> ur_connection = nullptr;
        std::unique_ptr<ur_rtde::RTDEControlInterface> ur_control = nullptr;
        std::unique_ptr<ur_rtde::RTDEReceiveInterface> ur_receive = nullptr;
        std::unique_ptr<ur_rtde::RTDEIOInterface> ur_IO = nullptr;
        
        void toggle_ur_connection();
        std::thread connect_thread;
        bool freemode = false;
        
        void ur_connect();
        void ur_disconnect();
        bool ur_isConnected();

        rw::math::Q path_end_Q = rw::math::Q(6);
        double laste = 0;
        double edt = 1;

        // Utility
        double Q_dist(rw::math::Q q1, rw::math::Q q2);
        std::vector<rw::math::Q> inverseKinematics(rw::math::Transform3D<> targetT);
        rw::math::Q nearest_Q(std::vector<rw::math::Q> Qs, rw::math::Q nearQ);
        std::vector<rw::math::Q> filterCollisionQs(std::vector<rw::math::Q> Qs);

        // Misc
        rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
};

#endif /*EITPLUGIN_HPP*/
