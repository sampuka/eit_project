#ifndef EITPLUGIN_HPP
#define EITPLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

// RobWorkStudio includes
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

// Qt
#include "ui_EITPlugin.h"

// RTDE
#include <rwhw/universalrobots_rtde/URRTDE.hpp>

// Standard includes
#include <exception>
#include <memory>
#include <thread>

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
        void sync_pressed(bool);

    private:
        void connect_ur();
        void move_ur(rw::math::Q q);

        // RobWork
        rw::models::WorkCell::Ptr rws_wc;
        rw::kinematics::State rws_state;
        rw::models::Device::Ptr UR_robot;

        // UR Robot
        const std::string ur_ip = "10.10.1.100";
        std::unique_ptr<rwhw::URRTDE> ur_connection = nullptr;
        std::thread connect_thread;
        bool freemode = false;

        // Misc
        rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
};

#endif /*EITPLUGIN_HPP*/
