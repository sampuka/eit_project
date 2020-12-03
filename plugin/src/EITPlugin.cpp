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

    home_Q = UR_robot->getQ(rws_state);
    rw::math::Transform3D<> homeT = base_frame->wTf(rws_state);

    // Find pick approach Q
    {
        rw::math::Transform3D<> pick_approach_T(
                rw::math::Vector3D<>(0.324, -0.500, 0.140),
                rw::math::RPY<>(-180*rw::math::Deg2Rad, 0, 180*rw::math::Deg2Rad));

        std::vector<rw::math::Q> possible_Qs = inverseKinematics(rw::math::inverse(homeT)*pick_approach_T);
        possible_Qs = filterCollisionQs(possible_Qs);

        pick_approach_Q = nearest_Q(possible_Qs, home_Q); // Nearest to home position
    }

    // Find pick Q
    {
        rw::math::Transform3D<> pick_T(
                rw::math::Vector3D<>(0.324, -0.500, 0.108),
                rw::math::RPY<>(-180*rw::math::Deg2Rad, 0, 180*rw::math::Deg2Rad));

        std::vector<rw::math::Q> possible_Qs = inverseKinematics(rw::math::inverse(homeT)*pick_T);
        possible_Qs = filterCollisionQs(possible_Qs);

        pick_Q = nearest_Q(possible_Qs, home_Q); // Nearest to home position
    }

    // Find place approach Qs
    place_approach_Qs.clear();

    for (unsigned int i = 0; i < place_position_count; i++)
    {
        rw::math::Transform3D<> approach_T(
                rw::math::Vector3D<>(x_lim1 + (x_lim2-x_lim1)*i/(place_position_count-1.0), 0.475, 0.280),
                rw::math::RPY<>(0, 0, 180*rw::math::Deg2Rad));

        std::vector<rw::math::Q> possible_Qs = inverseKinematics(rw::math::inverse(homeT)*approach_T);
        possible_Qs = filterCollisionQs(possible_Qs);

        rw::math::Q nearQ = nearest_Q(possible_Qs, home_Q); // Nearest to home position

        place_approach_Qs.push_back(nearQ);
    }

    // Find place Qs
    place_Qs.clear();

    for (unsigned int i = 0; i < place_position_count; i++)
    {
        rw::math::Transform3D<> place_T(
                rw::math::Vector3D<>(x_lim1 + (x_lim2-x_lim1)*i/(place_position_count-1.0), 0.475, 0.240),
                rw::math::RPY<>(0, 0, 180*rw::math::Deg2Rad));

        std::vector<rw::math::Q> possible_Qs = inverseKinematics(rw::math::inverse(homeT)*place_T);
        possible_Qs = filterCollisionQs(possible_Qs);

        rw::math::Q nearQ = nearest_Q(possible_Qs, home_Q); // Nearest to home position

        place_Qs.push_back(nearQ);
    }

    //UR_robot->setQ(place_approach_Qs[0], rws_state);
    //getRobWorkStudio()->setState(rws_state);
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
    connect_thread = std::thread(&EITPlugin::toggle_ur_connection, this);
}

void EITPlugin::button_freemode()
{
    std::cout << "Freemode button pressed!" << std::endl;

    if (ur_isConnected())
    {
        if (freemode)
        {
            freemode = false;
            ur_control->endTeachMode();
            ui_label_connection->setText("UR status: connected");
        }
        else
        {
            freemode = true;
            ur_control->teachMode();
            ui_label_connection->setText("UR status: connected (free mode)");
        }
    }
}

void EITPlugin::button_home()
{
    std::cout << "Home button pressed!" << std::endl;
    rw::math::Q from;
    rw::math::Q to = home_Q;
    double extend = 0.05;
    running = false;

    if(ui_checkbox_sync->isChecked()) {
        from = ur_receive->getActualQ();
        UR_robot->setQ(from, rws_state);
        getRobWorkStudio()->setState(rws_state);
      }
    else {
        from = UR_robot->getQ(rws_state);
      }
    create_trajectory(from, to, extend);
    running = true;
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
  std::vector<int> selection_vec({6,0,0,1,0,0,0});
  //Apply force in compliant direction
  std::vector<double> wrench({0.0,0.0,force,0.0,0.0,0.0});
  //Specify type - 1: Point, 2: Simple, 3: Motion
  int type = 2;
  //Specify limits in speed for compliant directions and a certain allowed deviation for noncompliant
    std::vector<double>  limits({0.1, 0.1, 0.15, 0.17, 0.17, 0.17});

  rw::math::RPY<double> rpy(feature.R());
  std::vector<double> f(6);
    for (int i = 0; i < 3; ++i) {
        f[i] = feature.P()[i];
        f[i+3] = rpy[i];
    }

  //Specify for how long robot should apply force
  std::chrono::milliseconds duration = std::chrono::milliseconds(500);

  if (ur_isConnected())
  {
      ur_control->forceMode(f, selection_vec, wrench, type, limits);
      std::this_thread::sleep_for(duration);
      ur_control->forceModeStop();
  }
}

void EITPlugin::button_start()
{
    std::cout << "Start button pressed!" << std::endl;

    trajectory.clear();
    trajectory_index = 0;
    rw::math::Q from;

    if (ui_checkbox_sync->isChecked())
    {
        std::cout << "Connected to real UR! setting simulation..." << std::endl;
        from = ur_receive->getActualQ();

        UR_robot->setQ(from, rws_state);
        getRobWorkStudio()->setState(rws_state);

    }
    else
    {
        from = UR_robot->getQ(rws_state);
        //rw::math::Q(6, 0.0,-1.0, -1.0, 0.0, 0.0, 0.0);
    }
    std::cout << "from: " << from << std::endl;
    rw::math::Q to = pick_approach_Q;

    rw::math::Math::seed();
    double extend = 0.05;
    running = false;
    path.clear();
    whole_path.clear();

    create_trajectory(from, home_Q, extend);
    whole_path.emplace_back(path, trash, false);

    for (unsigned int i = 0; i < 1; i++)
    {
        // Home to pick approach
        create_trajectory(home_Q, pick_approach_Q, extend);
        whole_path.emplace_back(path, trash, false);

        // Pick approach to pick
        create_trajectory(pick_approach_Q, pick_Q, extend);
        whole_path.emplace_back(path, trash, false);

        // Pick to pick approach
        create_trajectory(pick_Q, pick_approach_Q, extend);
        whole_path.emplace_back(path, trash, true);

        // Pick approach to place approach
        create_trajectory(pick_approach_Q, place_approach_Qs[i], extend);
        whole_path.emplace_back(path, trash, true);

        // Place approach to place
        create_trajectory(place_approach_Qs[i], place_Qs[i], extend);
        whole_path.emplace_back(path, trash, true);

        // Place to place approach
        create_trajectory(place_Qs[i], place_approach_Qs[i], extend);
        whole_path.emplace_back(path, trash, false);

        // Place approach to home
        create_trajectory(place_approach_Qs[i], home_Q, extend);
        whole_path.emplace_back(path, trash, false);
    }

    /*
    for (unsigned int i = 0; i < whole_path.size(); i++)
    {
        std::cout << i << ' ' << whole_path[i].traj.endTime() << ' ' << whole_path[i].traj.getSegmentsCount() << std::endl;

        for (double j = whole_path[i].traj.startTime(); j < whole_path[i].traj.endTime(); j+= 0.1)
        {
            std::cout << j << ": " << whole_path[i].traj.x(j) << std::endl;
        }
    }
    */

    /*for (int i = 0; i < 100; i++)
    {
        trajectory.emplace_back(std::make_pair(30, rw::math::Q(6, 0.0+i*0.01, -1.0+i*0.01, -1.0+i*0.01, 0.0+i*0.01, 0.0+i*0.01, 0.0+i*0.01)));
    }*/

    create_trajectory(from, to, extend);

    running = true;
    path_section = 0;
}

void EITPlugin::sync_pressed(bool checkbox_state) {
    if (checkbox_state) {
        if (!ur_isConnected()) {
            ui_checkbox_sync->setChecked(false);
            return;
        }
        // Checkbox checked
        //ui_checkbox_sync->setChecked(true);
        // Read real UR position

        rw::math::Q curr_q = ur_receive->getActualQ();
        //std::cout << "Current real position: " << curr_q << std::endl;
        // Set twin ur position
        UR_robot->setQ(curr_q, rws_state);
        rw::math::Q sim_q = UR_robot->getQ(rws_state);
        //std::cout << "Sim Q: " << sim_q << std::endl;
        getRobWorkStudio()->setState(rws_state);
        sim_q = UR_robot->getQ(rws_state);
        //std::cout << "Sim Q next state: " << sim_q << std::endl;
    }
}

void EITPlugin::control_loop()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> moved_ts; // Timestamp of last time we moved to new trajectory

    while (!should_shutdown)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /*if (!ur_isConnected())
        {
            ur_disconnect();
        }*/

        if (running)
        {
            if (ui_checkbox_sync->isChecked())
            {
                if(!ur_control->isProgramRunning()){
                    if (path_section == whole_path.size()-1)
                    {
                        running = false;
                        path_section = 0;
                        continue;
                    }
                    ur_control->moveJ(whole_path[path_section].path);
                    path_section++;
                }

                rw::math::Q curr_q(ur_receive->getActualQ());

                UR_robot->setQ(curr_q, rws_state);
                getRobWorkStudio()->setState(rws_state);
            }
            else
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> since_moved = now-moved_ts;

                if (since_moved.count() >= whole_path[path_section].traj.endTime())
                {
                    if (path_section == whole_path.size()-1)
                    {
                        running = false;
                        path_section = 0;
                        continue;
                    }

                    path_section++;
                    moved_ts = std::chrono::high_resolution_clock::now();
                }

                UR_robot->setQ(whole_path[path_section].traj.x(since_moved.count()), rws_state);
                getRobWorkStudio()->setState(rws_state);
            }
        }
        else
        {
            moved_ts = std::chrono::high_resolution_clock::now();
        }
    }
}

void EITPlugin::toggle_ur_connection()
{
    if ( ur_isConnected() ) ur_disconnect();
    else ur_connect();
}

void EITPlugin::ur_connect() {
    ur_control = std::make_unique<ur_rtde::RTDEControlInterface>(ur_ip);
    ur_receive = std::make_unique<ur_rtde::RTDEReceiveInterface>(ur_ip);
    ur_IO = std::make_unique<ur_rtde::RTDEIOInterface>(ur_ip);

    ui_label_connection->setText("UR status: connected");
    ui_button_connect_disconnect->setText("Disconnect");
    ui_button_freemode->setEnabled(true);
    ui_checkbox_sync->setEnabled(true);
}

void EITPlugin::ur_disconnect() {
    if (freemode)
    {
        freemode = false;
        ur_control->endTeachMode();
    }
    ur_control = nullptr;
    ur_receive = nullptr;
    ur_IO = nullptr;

    ui_label_connection->setText("UR status: disconnected");
    ui_button_connect_disconnect->setText("Connect");
    ui_button_freemode->setEnabled(false);
    ui_checkbox_sync->setEnabled(false);
}

bool EITPlugin::ur_isConnected(){
    return !((ur_control == nullptr) || (!ur_control->isConnected()) ||
           (ur_receive == nullptr) || (!ur_receive->isConnected()) ||
           (ur_IO == nullptr));
};

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
    if (Qs.size() == 0)
    {
        std::cout << "Error: finding nearest Q of empty list" << std::endl;
    }

    rw::math::Q best_Q;
    double best_Q_dist = std::numeric_limits<double>::max();

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

std::vector<rw::math::Q> EITPlugin::filterCollisionQs(std::vector<rw::math::Q> Qs)
{
    rw::proximity::CollisionDetector detector(rws_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    std::vector<rw::math::Q> colfree;

    rw::kinematics::State test_state = rws_state;

    for (const rw::math::Q &q : Qs)
    {
        UR_robot->setQ(q, test_state);

        if (!detector.inCollision(test_state))
        {
            colfree.push_back(q);
        }
    }

    return colfree;
}

void EITPlugin::create_trajectory(rw::math::Q from, rw::math::Q to, double extend)
{
  /*
   * TODO: Make path planning, from home to pick-up, open gripper and move down, close gripper, back to pick-up,
   * pick-up to approach point, linear down to place, open gripper, move up to approach point, close girpper, back to home
   *
   * How to sync with real robot?
   * Nice-to-have: apply force
   *
   * NOTE: I do not clear in this one
   * */
      rw::kinematics::State test_state = rws_state;
      UR_robot->setQ(from,test_state);
      getRobWorkStudio()->setState(test_state);
      rw::proximity::CollisionDetector detector(rws_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
      rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector, UR_robot, test_state);
      rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(UR_robot),constraint.getQConstraintPtr());
      rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
      rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);

       rw::proximity::CollisionDetector::QueryResult data;
       UR_robot->setQ(from, test_state);

       if (detector.inCollision(test_state, &data))
           RW_THROW("Initial configuration in collision! can not plan a path.");
       UR_robot->setQ (to, test_state);
       if (detector.inCollision(test_state, &data))
           RW_THROW("Final configuration in collision! can not plan a path.");

      //trajectory.clear();
      //trajectory_index = 0;

      rw::trajectory::QPath result;
      if (planner->query(from,to,result))
      {
           std::cout << "Planned path successfully with size " << result.size() << "." << std::endl;
      }


      path.clear();
      for(auto q: result){
        std::vector<double> p = q.toStdVector();
        p.push_back(0.5);
        p.push_back(0.5);
        p.push_back(0.025);
        path.push_back(p);
      }

      const int duration = 30;
      trash = rw::trajectory::InterpolatorTrajectory<rw::math::Q>();

      for (unsigned int i = 1; i < result.size(); i++) {
          rw::math::Q dQ = result[i-1] - result[i];
          double max_dq = 0;
          for (int j = 0; j < 6; j++)
              max_dq = (max_dq > std::abs(dQ[j]))? max_dq: std::abs(dQ[j]);

          double dt = (2.0 * max_dq);

          rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr traj = rw::ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q> (result[i-1], result[i], dt));

          trash.add(traj);
        }


      /*rw::trajectory::LinearInterpolator<rw::math::Q> linInt(from, to, duration);
      rw::trajectory::QPath tempQ;

      for(int i = 0; i < duration+1; i++)
      {
          trajectory.emplace_back(std::make_pair(30,linInt.x(i)));
      }*/
}


