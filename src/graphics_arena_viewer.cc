/**
 * @file graphics_arena_viewer.cc
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <iostream>
#include <string>
#include "src/graphics_arena_viewer.h"
#include "src/arena_params.h"
#include "src/rgb_color.h"
#include "src/robot.h"
#include "src/arena_entity.h"
#include "src/pose.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
GraphicsArenaViewer::GraphicsArenaViewer(
    const struct arena_params *const params,
    Arena * arena, Controller * controller) :
    GraphicsApp(
    params->x_dim + GUI_MENU_WIDTH + GUI_MENU_GAP * 2,
    params->y_dim,
    "Robot Simulation"),
controller_(controller),
arena_(arena),
robot_count_(5),
light_count_(0),
food_count_(0),
fear_count_(0),
light_sensitivity_(0.0f),
food_on_off_(0) {
auto *gui = new nanogui::FormHelper(screen());
nanogui::ref<nanogui::Window> window =
  gui->addWindow(
      Eigen::Vector2i(10 + GUI_MENU_GAP, 10),
      "Menu");

window->setLayout(new nanogui::GroupLayout());
gui->addGroup("Simulation Control");

playing_button_ =
gui->addButton(
  "Play",
  std::bind(&GraphicsArenaViewer::OnPlayingBtnPressed, this));
// adding a new button in the window
newgame_result_button_ =
gui->addButton(
  "Welcome",
  std::bind(&GraphicsArenaViewer::OnResultBtnPressed, this));
playing_button_->setFixedWidth(100);
gui->addGroup("Arena Configuration");

// Creating a panel impacts the layout. Widgets, sliders, buttons can be
// assigned to either the window or the panel.

// Making the function calls necessary to draw the different sliders of the GUI
DrawRobotSlider(window);
DrawLightSlider(window);
DrawFoodSlider(window);
DrawFoodOnOffSlider(window);
DrawFearSlider(window);
DrawLightSensorSlider(window);
screen()->performLayout();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

// This is the primary driver for state change in the arena.
// It will be called at each iteration of nanogui::mainloop()
void GraphicsArenaViewer::UpdateSimulation(double dt) {
  if (!paused_) {
    controller_->AdvanceTime(dt);
  }
}

// Function to deal with the situation once the game is over
void GraphicsArenaViewer::GameOver() {
  game_result_ = true;  // setting the flag true because we have a result
  paused_ = true;  // game should be paused because game over
  playing_button_->setCaption("New Game");
  newgame_result_button_->setCaption("Simulation Over");  // displaying result
}

/*******************************************************************************
 * Handlers for User Keyboard and Mouse Events
 ******************************************************************************/
//  Reset the arena when the New Game button is pressed
void GraphicsArenaViewer::OnResultBtnPressed() {
  if (!game_result_) {
    game_result_ = false;
    game_start_ = false;
    paused_ = true;
    playing_button_->setCaption("Play");
    controller_ -> AcceptCommunication(kNewGame);
  }
}

// Function to deal with the button presses during in the viewer
// during different stages of the game.
void GraphicsArenaViewer::OnPlayingBtnPressed() {
  // When button pressed before the start of the game
  // Sets the number of different entities and behavior for the simulation
  if (paused_ && !game_start_) {
    arena_ -> AddRobot(kRobot, robot_count_);  // adding desired no. of robots
    arena_ -> set_robot_count(robot_count_);
    arena_ -> AddLight(kLight, light_count_);  // adding desired no. of lights
    arena_ -> set_light_count(light_count_);
    if (food_on_off_ == 1) {
      arena_ -> AddFood(kFood, food_count_);  // adding food only if food is on
    }
    arena_ -> set_food_count(food_count_);
    // setting the number of robots that fear lights, food on off and the
    // the light sensitivity of the robots towards lights
    arena_ -> set_behavior_sensitivity_robot(fear_count_, light_sensitivity_,
                                                                food_on_off_);
    paused_ = !paused_;
    game_start_ = true;
    playing_button_->setCaption("Pause");
    newgame_result_button_->setCaption("New Game");
    controller_ -> AcceptCommunication(kPlay);
    return;
  } else if (game_result_) {  // buttons are pressed when game over
    game_result_ = false;
    game_start_ = false;
    paused_ = true;
    playing_button_->setCaption("Play");
    newgame_result_button_->setCaption("New Game");
    controller_ -> AcceptCommunication(kNewGame);
  } else if (!paused_) {  // when "Pause" is pressed during the game
    paused_ = true;
    playing_button_->setCaption("Play");
    controller_ -> AcceptCommunication(kPlay);
  } else {  // when "Play" is pressed to resume the game
    paused_ = false;
    playing_button_->setCaption("Pause");
    controller_ -> AcceptCommunication(kPause);
  }
}

/** OnSpecialKeyDown is called when the user presses down on one of the
  * special keys (e.g. the arrow keys).
  */
/**
 * @TODO: Check for arrow key presses using GLFW macros, then
 * convert to appropriate enum Communication and relay to controller
 */
void GraphicsArenaViewer::OnSpecialKeyDown(int key,
  __unused int scancode, __unused int modifiers) {
    Communication key_value = kNone;
    switch (key) {
      case GLFW_KEY_LEFT: key_value = kKeyLeft;
        break;
      case GLFW_KEY_RIGHT: key_value = kKeyRight;
        break;
      case GLFW_KEY_UP: key_value = kKeyUp;
        break;
      case GLFW_KEY_DOWN: key_value = kKeyDown;
        break;
      default: {}
    }
  controller_->AcceptCommunication(key_value);
}




/*******************************************************************************
 * Drawing of Entities in Arena
 ******************************************************************************/
void GraphicsArenaViewer::DrawRobot(NVGcontext *ctx,
                                      const Robot *const robot) {
  // translate and rotate all graphics calls that follow so that they are
  // centered, at the position and heading of this robot
  nvgSave(ctx);
  nvgTranslate(ctx,
              static_cast<float>(robot->get_pose().x),
              static_cast<float>(robot->get_pose().y));
  nvgRotate(ctx,
              static_cast<float>(robot->get_pose().theta * M_PI / 180.0));

  // robot's circle
  nvgBeginPath(ctx);
  nvgCircle(ctx, 0.0, 0.0, static_cast<float>(robot->get_radius()));
  nvgFillColor(ctx,
             nvgRGBA(robot->get_color().r, robot->get_color().g,
                    robot->get_color().b, 255));
  nvgFill(ctx);
  nvgStrokeColor(ctx, nvgRGBA(0, 0, 0, 255));
  nvgStroke(ctx);

  // robot id text label
  nvgSave(ctx);
  nvgRotate(ctx, static_cast<float>(M_PI / 2.0));
  nvgFillColor(ctx, nvgRGBA(0, 0, 0, 255));
  std::string r_ = "Robot:" + std::to_string(robot->get_behavior_flag());
  nvgText(ctx, 0.0, 4.0, r_.c_str(), nullptr);
  // nvgText(ctx, 0.0, 10.0, robot->get_lives(), nullptr);
  nvgRestore(ctx);
  nvgRestore(ctx);

  // Robot's left Light and Food Sensor
  nvgBeginPath(ctx);
  nvgCircle(ctx,
            static_cast<float>(robot->PoseLeftSensor().x),
            static_cast<float>(robot->PoseLeftSensor().y),
            static_cast<float>(4.0));
  nvgFillColor(ctx,
              nvgRGBA(255, 255, 255, 255));
  nvgFill(ctx);
  nvgStrokeColor(ctx, nvgRGBA(0, 0, 0, 255));
  nvgStroke(ctx);

  // Robot's right Light and Food Sensor
  nvgBeginPath(ctx);
  nvgCircle(ctx,
            static_cast<float>(robot->PoseRightSensor().x),
            static_cast<float>(robot->PoseRightSensor().y),
            static_cast<float>(4.0));
  nvgFillColor(ctx,
              nvgRGBA(255, 255, 255, 255));
  nvgFill(ctx);
  nvgStrokeColor(ctx, nvgRGBA(0, 0, 0, 255));
  nvgStroke(ctx);
}


void GraphicsArenaViewer::DrawArena(NVGcontext *ctx) {
  nvgBeginPath(ctx);
  // Creates new rectangle shaped sub-path.
  nvgRect(ctx, 0, 0, arena_->get_x_dim(), arena_->get_y_dim());
  nvgStrokeColor(ctx, nvgRGBA(255, 255, 255, 255));
  nvgStroke(ctx);
}

void GraphicsArenaViewer::DrawEntity(NVGcontext *ctx,
                                        const ArenaEntity *const entity) {
  // light's circle
  nvgBeginPath(ctx);
  nvgCircle(ctx,
            static_cast<float>(entity->get_pose().x),
            static_cast<float>(entity->get_pose().y),
            static_cast<float>(entity->get_radius()));
  nvgFillColor(ctx,
              nvgRGBA(entity->get_color().r, entity->get_color().g,
                        entity->get_color().b, 255));
  nvgFill(ctx);
  nvgStrokeColor(ctx, nvgRGBA(0, 0, 0, 255));
  nvgStroke(ctx);

  // light id text label
  nvgFillColor(ctx, nvgRGBA(0, 0, 0, 255));
  nvgText(ctx,
          static_cast<float>(entity->get_pose().x),
          static_cast<float>(entity->get_pose().y),
          entity->get_name().c_str(), nullptr);
}
void GraphicsArenaViewer::DrawUsingNanoVG(NVGcontext *ctx) {
  // initialize text rendering settings
  nvgFontSize(ctx, 18.0f);
  nvgFontFace(ctx, "sans-bold");
  nvgTextAlign(ctx, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  DrawArena(ctx);
  std::vector<class ArenaEntity *> entities_ = arena_->get_entities();
  for (auto &entity : entities_) {
    if (entity->get_type() != kRobot)
    DrawEntity(ctx, entity); /* for(i..) */
  }
  std::vector<class Robot *> robots_ = arena_->robot();
  for (auto &robot : robots_) {
    DrawRobot(ctx, robot); /* for(i..) */
  }
}
// Function for drawing the number of robots slider
void GraphicsArenaViewer::DrawRobotSlider(nanogui::ref<nanogui::Window>
                                                                   window) {
  nanogui::Widget *panel = new nanogui::Widget(window);
  new nanogui::Label(panel, "Number of Robots", "sans-bold");
  nanogui::Slider *slider = new nanogui::Slider(panel);
  // The starting value (range is from 0 to 1)
  // Note that below the displayed value is 10* slider value.
  slider->setValue(0.5f);
  slider->setFixedWidth(100);

  // Display the corresponding value of the slider in this textbox
  nanogui::TextBox *textBox = new nanogui::TextBox(panel);
  textBox->setFixedSize(nanogui::Vector2i(60, 25));
  textBox->setFontSize(20);
  textBox->setValue("5");

  // This is the lambda function called while the user is moving the slider
  slider->setCallback(
    [textBox](float value) {
      textBox->setValue(std::to_string(static_cast<
        int>(value*10)));
    });
  // This is the lambda function called once the user is no longer
  // manipulating the slider.
  slider->setFinalCallback(
    [&](float value) {
      robot_count_ = static_cast<int>(value*10);
    });
  panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
    nanogui::Alignment::Middle, 0, 15));
}
// Function for drawing the number of lights slider
void GraphicsArenaViewer::DrawLightSlider(nanogui::ref<nanogui::Window>
                                                                   window) {
  nanogui::Widget *panel = new nanogui::Widget(window);
  new nanogui::Label(panel, "Number of Lights", "sans-bold");
  nanogui::Slider *slider = new nanogui::Slider(panel);
  // The starting value (range is from 0 to 1)
  // Note that below the displayed value is 5* slider value.
  slider->setValue(0.0f);
  slider->setFixedWidth(100);

  // Display the corresponding value of the slider in this textbox
  nanogui::TextBox *textBox = new nanogui::TextBox(panel);
  textBox->setFixedSize(nanogui::Vector2i(60, 25));
  textBox->setFontSize(20);
  textBox->setValue("0");

  // This is the lambda function called while the user is moving the slider
  slider->setCallback(
    [textBox](float value) {
      textBox->setValue(std::to_string(static_cast<int>(value*5)));
    });
  // This is the lambda function called once the user is no longer
  // manipulating the slider
  slider->setFinalCallback(
    [&](float value) {
      light_count_ = static_cast<int>(value*5);
    });
  panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
    nanogui::Alignment::Middle, 0, 15));
}
// Function for drawing the number of food slider
void GraphicsArenaViewer::DrawFoodSlider(nanogui::ref<nanogui::Window>
                                                                   window) {
  nanogui::Widget *panel = new nanogui::Widget(window);
  new nanogui::Label(panel, "Number of Food", "sans-bold");
  nanogui::Slider *slider = new nanogui::Slider(panel);
  // The starting value (range is from 0 to 1)
  // Note that below the displayed value is 5* slider value.
  slider->setValue(0.0f);
  slider->setFixedWidth(100);

  // Display the corresponding value of the slider in this textbox
  nanogui::TextBox *textBox = new nanogui::TextBox(panel);
  textBox->setFixedSize(nanogui::Vector2i(60, 25));
  textBox->setFontSize(20);
  textBox->setValue("0");

  // This is the lambda function called while the user is moving the slider
  slider->setCallback(
    [textBox](float value) {
      textBox->setValue(std::to_string(static_cast<int>(value*5)));
    });
  // This is the lambda function called once the user is no longer
  // manipulating the slider
  slider->setFinalCallback(
    [&](float value) {
      food_count_ = static_cast<int>(value*5);
    });
  panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
    nanogui::Alignment::Middle, 0, 15));
}

// Function for drawing the fear slider for robots i.e. percentage of robots
// that fear lights.
void GraphicsArenaViewer::DrawFearSlider(nanogui::ref<nanogui::Window>
                                                                   window) {
  nanogui::Widget *panel = new nanogui::Widget(window);
  new nanogui::Label(panel, "Percentage Fear", "sans-bold");
  nanogui::Slider *slider = new nanogui::Slider(panel);
  // The starting value (range is from 0 to 1)
  // Note that below the displayed value is 100* slider value.
  slider->setValue(0.0f);
  slider->setFixedWidth(100);

  // Display the corresponding value of the slider in this textbox
  nanogui::TextBox *textBox = new nanogui::TextBox(panel);
  textBox->setFixedSize(nanogui::Vector2i(70, 30));
  textBox->setFontSize(20);
  textBox->setValue("0");
  textBox->setUnits("%");

  // This is the lambda function called while the user is moving the slider
  slider->setCallback(
    [textBox](float value) {
      textBox->setValue(std::to_string(static_cast<int>(value*100)));
    });
  // This is the lambda function called once the user is no longer
  // manipulating the slider
  slider->setFinalCallback(
    [&](float value) {
      fear_count_ = static_cast<int>(value*100);
      fear_count_ = static_cast<int>(fear_count_ * robot_count_/ 100);
    });
  panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
    nanogui::Alignment::Middle, 0, 15));
}

// Function that draws the slider for the light sensitivity of robots towards
// lights.
void GraphicsArenaViewer::DrawLightSensorSlider(nanogui::ref<nanogui::Window>
                                                                   window) {
  nanogui::Widget *panel = new nanogui::Widget(window);
  new nanogui::Label(panel, "LightSensor Sensitivity", "sans-bold");
  nanogui::Slider *slider = new nanogui::Slider(panel);
  // The starting value (range is from 0 to 1)
  slider->setValue(0.0f);
  slider->setFixedWidth(100);

  // Display the corresponding value of the slider in this textbox
  nanogui::TextBox *textBox = new nanogui::TextBox(panel);
  textBox->setFixedSize(nanogui::Vector2i(60, 25));
  textBox->setFontSize(20);
  textBox->setValue("1");

  // This is the lambda function called while the user is moving the slider
  slider->setCallback(
    [textBox](float value) {
      textBox->setValue(std::to_string(1+value*0.10));
    });
  // This is the lambda function called once the user is no longer
  // manipulating the slider
  slider->setFinalCallback(
    [&](float value) {
      light_sensitivity_ = (1+value*0.10);
    });
  panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
    nanogui::Alignment::Middle, 0, 15));
}


// Function for drawing the food on off slider i.e. gives the user the option
// to choose if food is on or off for the simulation
void GraphicsArenaViewer::DrawFoodOnOffSlider(nanogui::ref<nanogui::Window>
                                                                   window) {
  nanogui::Widget *panel = new nanogui::Widget(window);
  new nanogui::Label(panel, "Food On Off", "sans-bold");
  nanogui::Slider *slider = new nanogui::Slider(panel);
  // The starting value (range is from 0 to 1)
  slider->setValue(0.5f);
  slider->setFixedWidth(100);

  // Display the corresponding value of the slider in this textbox
  nanogui::TextBox *textBox = new nanogui::TextBox(panel);
  textBox->setFixedSize(nanogui::Vector2i(60, 25));
  textBox->setFontSize(20);
  textBox->setValue("On");

  // This is the lambda function called while the user is moving the slider
  slider->setCallback(
    [textBox](float value) {
      int value_onoff = value * 2;
      if (value_onoff == 0) {
        textBox->setValue("Off");
      } else {
        textBox->setValue("On");
      }
    });
  // This is the lambda function called once the user is no longer
  // manipulating the slider
  slider->setFinalCallback(
    [&](float value) {
      food_on_off_ = static_cast<int>(value*2);
    });
  panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
    nanogui::Alignment::Middle, 0, 15));
}
NAMESPACE_END(csci3081);
