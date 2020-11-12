#include "EITPlugin.hpp"

EITPlugin::EITPlugin():
  //TODO: Make GUI thread and ur communication thread

    RobWorkStudioPlugin("EITPluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // Connect UI components to member functions
    connect(ui_button_connect_disconnect, SIGNAL(pressed()), this, SLOT(button_connect_disconnect()));
    connect(ui_button_freemode, SIGNAL(pressed()), this, SLOT(button_freemode()));
    connect(ui_button_home, SIGNAL(pressed()), this, SLOT(button_home()));
    connect(ui_button_start, SIGNAL(pressed()), this, SLOT(button_start()));
    connect(ui_checkbox_sync, SIGNAL(clicked(bool)), this, SLOT(sync_pressed(bool)));
}

EITPlugin::~EITPlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void EITPlugin::initialize()
{
    log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&EITPlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Get path to projectf from environment
    char* projectpath = std::getenv("EITDIR");

    if (projectpath == NULL)
    {
        std::cerr << "EITDIR environment variable not set! Perform \"export EITDIR=/home/user/eit_project/\", with the correct path." << std::endl;
    }
    else
    {
        rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(std::string(projectpath) + "/workcell/Scene.wc.xml");

        if (wc == nullptr)
        {
            std::cerr << "Unable to autoload workcell! Maybe EITDIR environment variable not set correctly?" << std::endl;
        }
        else
        {
            getRobWorkStudio()->setWorkCell(wc);
        }
    }

    control_loop_thread = std::thread(&EITPlugin::control_loop, this);
}

void EITPlugin::open(rw::models::WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    rws_wc = workcell;
    rws_state = rws_wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (rws_wc != NULL)
    {
        // Add the texture render to this workcell if there is a frame for texture
        rw::kinematics::Frame* textureFrame = rws_wc->findFrame("MarkerTexture");
        if (textureFrame != NULL)
        {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
        }

        // Add the background render to this workcell if there is a frame for texture
        rw::kinematics::Frame* bgFrame = rws_wc->findFrame("Background");
        if (bgFrame != NULL)
        {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
        }

        UR_robot = rws_wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
        base_frame = rws_wc->findFrame<rw::kinematics::Frame>("UR-6-85-5-A.BaseMov");

        if (UR_robot == nullptr)
            std::cerr << "Could not find UR!" << std::endl;

        if (base_frame == nullptr)
            std::cerr << "Could not find base frame!" << std::endl;
    }

    // Find place Qs
    place_approach_Qs.clear();
    place_Qs.clear();

    rw::math::Q homeQ = UR_robot->getQ(rws_state);
    rw::math::Transform3D<> homeT = base_frame->wTf(rws_state);

    for (unsigned int i = 0; i < place_position_count; i++)
    {
        rw::math::Transform3D<> approach_T(
                rw::math::Vector3D<>(x_lim1 + (x_lim2-x_lim1)*i/(place_position_count-1.0), 0.465, 0.268),
                rw::math::RPY<>(0, 0, 180*rw::math::Deg2Rad));

        std::vector<rw::math::Q> possible_Qs = inverseKinematics(rw::math::inverse(homeT)*approach_T);

        rw::math::Q nearQ = nearest_Q(possible_Qs, homeQ); // Nearest to home position

        place_approach_Qs.push_back(nearQ);
    }

    for (unsigned int i = 0; i < place_position_count; i++)
    {
        rw::math::Transform3D<> place_T(
                rw::math::Vector3D<>(x_lim1 + (x_lim2-x_lim1)*i/(place_position_count-1.0), 0.465, 0.238),
                rw::math::RPY<>(0, 0, 180*rw::math::Deg2Rad));

        std::vector<rw::math::Q> possible_Qs = inverseKinematics(rw::math::inverse(homeT)*place_T);

        rw::math::Q nearQ = nearest_Q(possible_Qs, homeQ); // Nearest to home position

        place_Qs.push_back(nearQ);
    }

    UR_robot->setQ(place_approach_Qs.at(1), rws_state);
    getRobWorkStudio()->setState(rws_state);
}

void EITPlugin::close()
{
    log().info() << "CLOSE" << "\n";

    // Remove the texture render
    rw::kinematics::Frame* textureFrame = rws_wc->findFrame("MarkerTexture");
    if (textureFrame != NULL)
    {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
    }

    // Remove the background render
    rw::kinematics::Frame* bgFrame = rws_wc->findFrame("Background");
    if (bgFrame != NULL)
    {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
    }

    rws_wc = NULL;
}

void EITPlugin::stateChangedListener(const rw::kinematics::State& state)
{
    rws_state = state;
}

void EITPlugin::button_connect_disconnect()
{
    std::cout << "Connect button pressed!" << std::endl;

    if (connect_thread.joinable()) {
        connect_thread.join();
    }
    connect_thread = std::thread(&EITPlugin::connect_ur, this);
}

void EITPlugin::button_freemode()
{
    std::cout << "Freemode button pressed!" << std::endl;

    if ((ur_connection == nullptr) || (!ur_connection->isConnected()))
    {
        if (freemode)
        {
            freemode = false;
            ur_connection->endTeachMode();
            ui_label_connection->setText("UR status: connected");
        }
        else
        {
            freemode = true;
            ur_connection->teachMode();
            ui_label_connection->setText("UR status: connected (free mode)");
        }
    }
}

void EITPlugin::button_home()
{
    std::cout << "Home button pressed!" << std::endl;

    rw::math::Q q(6, 0,-1.6,-1.6,0,0,0);
    move_ur(q);
}

void EITPlugin::apply_force(double force)
{
  //TODO: Should only be in force mode while placing! How can we do that?

  std::cout << "Applying force!" << std::endl;
  // Force mode on real robot, nothing in RWS.

  //SETUP force mode
  //Force frame relative to base frame
  rw::math::Transform3D feature = UR_robot->getEnd()->getTransform(rws_state);
  //Compliant in Z-axis, selection vector (X,Y,Z,R,P,Y)
  rw::math::Q selection_vec(6, 0,0,1,0,0,0);
  //Apply force in compliant direction
  rw::math::Wrench6D wrench(0.0,0.0,force,0.0,0.0,0.0);
  //Specify type - 1: Point, 2: Simple, 3: Motion
  int type = 2;
  //Specify limits in speed for compliant directions and a certain allowed deviation for noncompliant
  rw::math::Q limits(0.1, 0.1, 0.15, 0.17, 0.17, 0.17);

  //Specify for how long robot should apply force
  std::chrono::milliseconds duration = std::chrono::milliseconds(500);

  if (ur_connection != nullptr && ur_connection->isConnected())
  {
      ur_connection->forceMode(feature, selection_vec, wrench, type, limits);
      std::this_thread::sleep_for(duration);
      /*
      std::chrono::time_point<std::chrono::high_resolution_clock> finished = std::chrono::high_resolution_clock::now() + duration;

      while (std::chrono::system_clock::now() < finished) {
          ur_connection->forceMode(feature, selection_vec, wrench, type, limits);
      }
      */
      ur_connection->forceModeStop();
  }

}

void EITPlugin::button_start()
{
    std::cout << "Start button pressed!" << std::endl;

    trajectory.clear();
    trajectory_index = 0;

    for (int i = 0; i < 100; i++)
    {
        trajectory.emplace_back(std::make_pair(30, rw::math::Q(6, 0.0+i*0.01, -1.0+i*0.01, -1.0+i*0.01, 0.0+i*0.01, 0.0+i*0.01, 0.0+i*0.01)));
    }

    running = true;
}

void EITPlugin::sync_pressed(bool checkbox_state)
{
    if (checkbox_state)
    {
        if (ur_connection == nullptr || !ur_connection->isConnected())
        {
            ui_checkbox_sync->setChecked(false);
        }
        // Checkbox checked
        // Read real UR position
        // Set twin ur position
    }
    else
    {
        // Checkbox unchecked
    }
}

void EITPlugin::control_loop()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> moved_ts; // Timestamp of last time we moved to new trajectory

    while (!should_shutdown)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (ur_connection != nullptr && !ur_connection->isConnected())
        {
            ur_connection = nullptr;
            freemode = false;
            ui_label_connection->setText("UR status: disconnected");
            ui_button_connect_disconnect->setText("Connect");
            ui_checkbox_sync->setChecked(false);
            ui_checkbox_sync->setEnabled(false);
        }

        if (running && !trajectory.empty())
        {
            if (ui_checkbox_sync->isChecked())
            {
                rw::math::Q curr_q = ur_connection->getActualQ();

                UR_robot->setQ(curr_q, rws_state);
                getRobWorkStudio()->setState(rws_state);
            }
            else
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> since_moved = now-moved_ts;
                //std::cout << '\n';
                //std::cout /*<< moved_ts.count() << '\n' << now.count() << '\n'*/ << since_moved.count() << std::endl;

                if (
                        (trajectory_index == 0) ||
                        (std::chrono::duration_cast<std::chrono::milliseconds>(since_moved).count() > trajectory.at(trajectory_index-1).first)
                   )
                {
                    if (trajectory_index == trajectory.size())
                    {
                        running = false;
                        continue;
                    }

                    UR_robot->setQ(trajectory.at(trajectory_index).second, rws_state);
                    getRobWorkStudio()->setState(rws_state);

                    trajectory_index++;

                    moved_ts = std::chrono::high_resolution_clock::now();
                }
            }
        }
        else
        {
            moved_ts = std::chrono::high_resolution_clock::now();
        }
    }
}

void EITPlugin::connect_ur()
{
    if ((ur_connection == nullptr) || (!ur_connection->isConnected()))
    {
        ur_connection = std::make_unique<rwhw::URRTDE>(ur_ip);
        ui_label_connection->setText("UR status: connected");
        ui_button_connect_disconnect->setText("Disconnect");
        ui_button_freemode->setEnabled(true);
        ui_checkbox_sync->setEnabled(true);
    }
    else
    {
        if (freemode)
        {
            freemode = false;
            ur_connection->endTeachMode();
        }
        ur_connection = nullptr;
        ui_label_connection->setText("UR status: disconnected");
        ui_button_connect_disconnect->setText("Connect");
        ui_button_freemode->setEnabled(false);
        ui_checkbox_sync->setEnabled(false);
    }
}

void EITPlugin::move_ur(rw::math::Q q)
{
    std::cout << "Trying to move to " << q << "..."<< std::endl;
    if (ui_checkbox_sync->checkState()) {
        //TODO: As Mathias, make trajectory between two configs and move.
        rw::math::Q from = UR_robot->getQ(rws_state);
        rw::math::Q to = q;
      }
}

double EITPlugin::Q_dist(rw::math::Q q1, rw::math::Q q2)
{
    return (q2-q1).norm2();
}

std::vector<rw::math::Q> EITPlugin::inverseKinematics(rw::math::Transform3D<> targetT)
{
    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSolver = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(UR_robot, rws_state));

    return closedFormSolver->solve(targetT, rws_state);
}

rw::math::Q EITPlugin::nearest_Q(std::vector<rw::math::Q> Qs, rw::math::Q nearQ)
{
    rw::math::Q best_Q;
    double best_Q_dist = 99999999;

    for (const auto& Q : Qs)
    {
        double dist = Q_dist(nearQ, Q);
        if (dist < best_Q_dist)
        {
            best_Q_dist = dist;
            best_Q = Q;
        }
    }

    return best_Q;
}
