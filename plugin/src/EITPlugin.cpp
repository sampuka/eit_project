#include "EITPlugin.hpp"

EITPlugin::EITPlugin():
  //TODO: Make GUI thread and ur communication thread

    RobWorkStudioPlugin("EITPluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // Connect UI components to member functions
    connect(ui_home_button, SIGNAL(pressed()), this, SLOT(home_button()));

    connect(ui_reconnect_button, SIGNAL(pressed()), this, SLOT(reconnect_button()));

    connect(ui_checkbox_sync, SIGNAL(clicked(bool)), this, SLOT(sync_pressed(bool)));

    //connect(_btn_im    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    paul = new rwhw::URRTDE ("10.42.0.100");
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

        UR_robot = rws_wc->findDevice("UR-6-85-5-A");

        if (UR_robot == nullptr)
            std::cerr << "Could not find UR!" << std::endl;
    }
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

void EITPlugin::home_button()
{
    std::cout << "Home button pressed!" << std::endl;
    rw::math::Q q(6, 0,-1.6,-1.6,0,0,0);

    if (paul->isConnected()) {
      if (thread_name.joinable()) {
      thread_name.join();
        }
      thread_name = std::thread(&EITPlugin::move_ur, this, q);
      //move_ur(q);
      }

}

void EITPlugin::reconnect_button()
{
  std::cout << "Reconnect button pressed!" << std::endl;
  if (thread_name.joinable()) {
  thread_name.join();
    }
  thread_name = std::thread(&EITPlugin::reconnect_ur, this);
}

void EITPlugin::reconnect_ur()
{
  std::cout << "Paul reconnects..." << std::endl;
  if (paul->isConnected()) {
      std::cout << "Paul feels connected. Reset connection..." << std::endl;
      delete paul;
      paul = new rwhw::URRTDE ("10.42.0.100");
    }
  else {
      std::cout << "Paul feels disconnected. Reconnects..." << std::endl;
      paul->reconnect();
    }
  std::cout << "Connected!" << std::endl;
}

void EITPlugin::move_ur(rw::math::Q q)
{
  std::cout << "Paul moves..." << std::endl;
  try {
    paul->moveJ(q,0.1,0.1);
  } catch (std::exception& e) {
      std::cout << e.what() << std::endl;
  }
  std::cout << "...Paul finished moving." << std::endl;
  //paul->stopRobot(); //TEST if it does not work remove

}

void EITPlugin::sync_pressed(bool checkbox_state)
{
    if (checkbox_state)
    {
        // Checkbox checked
        // Read real UR position
        // Set twin ur position
    }
    else
    {
        // Checkbox unchecked
    }
}
