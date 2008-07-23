#ifndef REFEREEMM_MAIN_WINDOW
#define REFEREEMM_MAIN_WINDOW

#include <gtkmm.h>
#include "gamecontrol.h"

class Refereemm_Main_Window : public Gtk::Window
{
public:
   Refereemm_Main_Window(GameControl&);
   virtual ~Refereemm_Main_Window();
   
   // signale
   virtual void on_exit_clicked();
   virtual void on_load_config();

   virtual void on_start_button();
   virtual void on_stop_button();
   virtual void on_cancel();
   virtual void on_halt();
   virtual void on_ready();
   
   
   virtual void on_yellow_goal();
   virtual void on_yellow_subgoal();
   virtual void on_yellow_kickoff();
   virtual void on_yellow_freekick();
   virtual void on_yellow_penalty();
   virtual void on_yellow_indirect_freekick();
   virtual void on_yellow_timeout_start();
   virtual void on_yellow_timeout_stop();
   virtual void on_yellow_yellowcard();
   virtual void on_yellow_redcard();

   virtual void on_blue_goal();
   virtual void on_blue_subgoal();
   virtual void on_blue_kickoff();
   virtual void on_blue_freekick();
   virtual void on_blue_penalty();
   virtual void on_blue_indirect_freekick();
   virtual void on_blue_timeout_start();
   virtual void on_blue_timeout_stop();
   virtual void on_blue_yellowcard();
   virtual void on_blue_redcard();
   virtual void on_teamname_yellow();
   virtual void on_teamname_blue();

   virtual void on_toggle_enable_commands();

   // Idle Function
   virtual void idle();

   // Gui Sensitive
   void set_active_widgets(const EnableState&);
   
   
protected:
   // Non GUI Elements
   GameControl& gamecontrol;

   // Elements
   Gtk::CheckButton enable_commands_but;
   Gtk::Button start_but;
   Gtk::Button stop_but;
   Gtk::MenuBar menu_bar;
   Gtk::Menu file_m, config_m;
   Gtk::Label game_status_label;
   Gtk::Label time_label;
   
   Gtk::Frame goal_frame;
   Gtk::VBox  goal_vbox;
   Gtk::Label yellow_goal;
   Gtk::Label blue_goal;
   Gtk::Label vs;
   Gtk::Entry teamname_yellow;
   Gtk::Entry teamname_blue;
   
   Gtk::HBox game_status_hbox;
   Gtk::VBox game_status_vbox;

   Gtk::Frame game_control_frame;
   Gtk::VButtonBox game_control_box;
   Gtk::HBox yellow_goal_box;
   Gtk::HBox blue_goal_box;   
   Gtk::Button yellow_goal_but;
   Gtk::Button blue_goal_but;
   Gtk::Button yellow_subgoal_but;
   Gtk::Button blue_subgoal_but;
   Gtk::Button cancel_but;
   Gtk::Button halt_but;
   Gtk::Button ready_but;
   
   Gtk::Frame yellow_frame;
   Gtk::Button yellow_kickoff_but;
   Gtk::Button yellow_freekick_but;
   Gtk::Button yellow_penalty_but;
   Gtk::Button yellow_indirect_freekick_but;
   Gtk::Button yellow_timeout_start_but;
//   Gtk::Button yellow_timeout_stop_but;
   Gtk::Label yellow_timeout_time_label;
   Gtk::Label yellow_timeout_time_text;
   Gtk::Label yellow_timeouts_left_label;
   Gtk::Label yellow_timeouts_left_text;
   Gtk::Button yellow_yellowcard_but;
   Gtk::Button yellow_redcard_but;
   Gtk::Label yellow_card_label;

   Gtk::Frame blue_frame;   
   Gtk::Button blue_kickoff_but;
   Gtk::Button blue_freekick_but;
   Gtk::Button blue_penalty_but;
   Gtk::Button blue_indirect_freekick_but;
   Gtk::Button blue_timeout_start_but;
//   Gtk::Button blue_timeout_stop_but;
   Gtk::Label blue_timeout_time_label;
   Gtk::Label blue_timeout_time_text;
   Gtk::Label blue_timeouts_left_label;
   Gtk::Label blue_timeouts_left_text;
   Gtk::Button blue_yellowcard_but;
   Gtk::Button blue_redcard_but;
   Gtk::Label blue_card_label;


   // Arrange Widget
   Gtk::VBox big_vbox;
   Gtk::HBox halt_stop_hbox;
   Gtk::HBox start_ready_hbox;
   Gtk::HBox goal_hbox;
   Gtk::HBox teamname_hbox;
   Gtk::HBox team_hbox;
   Gtk::Table yellow_team_table;
   Gtk::Table blue_team_table;
};

#endif




