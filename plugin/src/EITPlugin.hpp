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

class EITPlugin: public rws::RobWorkStudioPlugin, private Ui::EITPlugin
{
    Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")

    public:
        EITPlugin();
        virtual ~EITPlugin();

        virtual void open(rw::models::WorkCell* workcell);
        virtual void close();
        virtual void initialize();

    private slots:
        void stateChangedListener(const rw::kinematics::State& state);

        void home_button();
        void sync_pressed(bool);

    private:
        // RobWork
        rw::models::WorkCell::Ptr rws_wc;
        rw::kinematics::State rws_state;
        rw::models::Device::Ptr UR_robot;

        // UR Robot
        std::string UR_IP = "1.2.3.4";

        // Misc
        rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
};

#endif /*EITPLUGIN_HPP*/
