#include <cmath>
#include <iostream>
#include "refereemm.h"
#include "serial.h"
#include "gamecontrol.h"
#include "gameinfo.h"
#ifdef WIN32
#include "getopt.h"
#endif

Refereemm_Main_Window::Refereemm_Main_Window(GameControl& gc_): 
    Gtk::Window(),
    gamecontrol(gc_),
    start_but("Force Start (KP_5)"),
    stop_but("Stop Game (KP_0)"),
    game_status_label("Stopped"),
    time_label("00.00:00"),
    goal_frame(" Yellow vs. Blue "),
    yellow_goal("00"),
    blue_goal("00"),
    vs(":"),
    game_control_frame("Game Status"),
    game_control_box(Gtk::BUTTONBOX_DEFAULT_STYLE, 10 ),
    yellow_goal_but("Goal Yellow! (KP_Div)"),
    blue_goal_but("Goal Blue! (KP_Mult)"),
    yellow_subgoal_but("-"),
    blue_subgoal_but("-"),
    cancel_but("Cancel"),
    halt_but("Halt (KP_Point)"),
    ready_but("Normal Start (KP_Enter)"),
    yellow_frame("Yellow Team"),
    yellow_kickoff_but("Kickoff (KP_1)"),
    yellow_freekick_but("Freekick (KP_7)"),
    yellow_penalty_but("Penalty"),
    yellow_indirect_freekick_but("Indirect (KP_4)"),
    yellow_timeout_start_but("Timeout Start"),
    yellow_timeout_time_label("Timeout Clock: "),
    yellow_timeouts_left_label("Timeouts left: "),
    yellow_yellowcard_but("Yellow Card"),
    yellow_redcard_but("Red card"),
    yellow_card_label("Red/Yellow Card"),
    blue_frame("Blue Team"),
    blue_kickoff_but("Kickoff (KP_3)"),
    blue_freekick_but("Freekick (KP_9)"),
    blue_penalty_but("Penalty"),
    blue_indirect_freekick_but("Indirect (KP_6)"),
    blue_timeout_start_but("Timeout Start"),
    blue_timeout_time_label("Timeout Clock: "),
    blue_timeouts_left_label("Timeouts left: "),
    blue_yellowcard_but("Yellow Card"),
    blue_redcard_but("Red card"),


    //yellow_timeout_stop_but("Timeout Stop"),
    blue_card_label("Red/Yellow Card")
    //blue_timeout_stop_but("Timeout Stop"),
{
   set_default_size(500,700);
   set_title("Small Size League - Referee Box");
   
   // Add Accelerator Buttons
   stop_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_0,Gdk::ModifierType(0),Gtk::AccelFlags(0));
   yellow_kickoff_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_1,Gdk::ModifierType(0),Gtk::AccelFlags(0));
   blue_kickoff_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_3,Gdk::ModifierType(0),Gtk::AccelFlags(0));
   yellow_indirect_freekick_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_4,Gdk::ModifierType(0),Gtk::AccelFlags(0));
   start_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_5,Gdk::ModifierType(0),Gtk::AccelFlags(0));
   blue_indirect_freekick_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_6,Gdk::ModifierType(0),Gtk::AccelFlags(0));
   yellow_freekick_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_7,Gdk::ModifierType(0),Gtk::AccelFlags(0));
   blue_freekick_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_9,Gdk::ModifierType(0),Gtk::AccelFlags(0));

   ready_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_Enter,Gdk::ModifierType(0),Gtk::AccelFlags(0));
   halt_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_Decimal,Gdk::ModifierType(0),Gtk::AccelFlags(0));

   yellow_goal_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_Divide,Gdk::ModifierType(0),Gtk::AccelFlags(0));
   blue_goal_but.add_accelerator( "activate",get_accel_group(),
                             GDK_KP_Multiply,Gdk::ModifierType(0),Gtk::AccelFlags(0));


   // Menu
   Gtk::Menu::MenuList& menulist = config_m.items();
   menulist.push_back( Gtk::Menu_Helpers::MenuElem("Load Config",
                       sigc::mem_fun(*this, &Refereemm_Main_Window::on_load_config)));
   menulist.push_back( Gtk::Menu_Helpers::CheckMenuElem("Enable Commands",
                       sigc::mem_fun(*this, &Refereemm_Main_Window::on_toggle_enable_commands)));
   menulist = file_m.items();
   menulist.push_back( Gtk::Menu_Helpers::MenuElem("Quit",
                       sigc::mem_fun(*this, &Refereemm_Main_Window::on_exit_clicked) ) );

   menu_bar.items().push_back( Gtk::Menu_Helpers::MenuElem("_Referee", file_m));
   menu_bar.items().push_back( Gtk::Menu_Helpers::MenuElem("_Config", config_m));

   // Connecting the one million Signals
//   enable_commands_but.signal_clicked():connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_toggle_enable_commands));
   start_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_start_button));
   //   start_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_start_button));
   stop_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_stop_button));
   halt_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_halt));   
   cancel_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_cancel));
   ready_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_ready));

   yellow_goal_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_goal));
   blue_goal_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_goal));

   yellow_subgoal_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_subgoal));
   blue_subgoal_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_subgoal));

   yellow_timeout_start_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_timeout_start));
   //   yellow_timeout_stop_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_timeout_stop));
   yellow_kickoff_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_kickoff));
   yellow_freekick_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_freekick));
   yellow_penalty_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_penalty));
   yellow_indirect_freekick_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_indirect_freekick));
   yellow_yellowcard_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_yellowcard));
   yellow_redcard_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_yellow_redcard));

   blue_timeout_start_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_timeout_start));
   //   blue_timeout_stop_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_timeout_stop));
   blue_kickoff_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_kickoff));
   blue_freekick_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_freekick));
   blue_penalty_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_penalty));
   blue_indirect_freekick_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_indirect_freekick));
   blue_yellowcard_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_yellowcard));
   blue_redcard_but.signal_clicked().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_blue_redcard));

   //teamname changed
   teamname_yellow.signal_changed().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_teamname_yellow));
teamname_blue.signal_changed().connect(sigc::mem_fun(*this, &Refereemm_Main_Window::on_teamname_blue));
   

   
   // Connecting the idle Signal
   Glib::signal_idle().connect( sigc::bind_return(sigc::mem_fun(*this, &Refereemm_Main_Window::idle), true));

   // Team Frames
   yellow_team_table.resize(7,2);
   yellow_team_table.set_row_spacings(10);
   yellow_team_table.set_col_spacings(10);
   yellow_team_table.attach(yellow_timeout_start_but, 0,1,0,1, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   //   yellow_team_table.attach(yellow_timeout_stop_but, 1,2,0,1, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_timeout_time_label, 0,1,1,2, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_timeout_time_text, 1,2,1,2, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_timeouts_left_label, 0,1,2,3, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_timeouts_left_text, 1,2,2,3, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_kickoff_but, 0,1,3,4, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_penalty_but, 1,2,3,4, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_freekick_but, 0,1,4,5, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_indirect_freekick_but, 1,2,4,5, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);

   yellow_team_table.attach(yellow_card_label, 0,1,5,6, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_yellowcard_but, 0,1,6,7, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   yellow_team_table.attach(yellow_redcard_but, 1,2,6,7, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);

   yellow_frame.add(yellow_team_table);
   yellow_frame.modify_bg(Gtk::STATE_NORMAL , Gdk::Color("yellow"));

   team_hbox.pack_start( yellow_frame, Gtk::PACK_EXPAND_WIDGET, 20 );
   
   blue_team_table.resize(7,2);
   blue_team_table.set_row_spacings(10);
   blue_team_table.set_col_spacings(10);
   blue_team_table.attach(blue_timeout_start_but, 0,1,0,1, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   //   blue_team_table.attach(blue_timeout_stop_but, 1,2,0,1, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_timeout_time_label, 0,1,1,2, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_timeout_time_text, 1,2,1,2, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_timeouts_left_label, 0,1,2,3, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_timeouts_left_text, 1,2,2,3, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_kickoff_but, 0,1,3,4, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_penalty_but, 1,2,3,4, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_freekick_but, 0,1,4,5, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_indirect_freekick_but, 1,2,4,5, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);

   blue_team_table.attach(blue_card_label, 0,1,5,6, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_yellowcard_but, 0,1,6,7, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);
   blue_team_table.attach(blue_redcard_but, 1,2,6,7, Gtk::EXPAND| Gtk::FILL, Gtk::EXPAND| Gtk::FILL);


   blue_frame.add(blue_team_table);
   blue_frame.modify_bg(Gtk::STATE_NORMAL , Gdk::Color("blue"));
   
   team_hbox.pack_start( blue_frame, Gtk::PACK_EXPAND_WIDGET, 20 );
   
   // Start stop
   halt_stop_hbox.pack_start(halt_but, Gtk::PACK_EXPAND_WIDGET, 20);
   halt_stop_hbox.pack_start(stop_but, Gtk::PACK_EXPAND_WIDGET, 20 );
   start_ready_hbox.pack_start(start_but, Gtk::PACK_EXPAND_WIDGET, 20 );
   start_ready_hbox.pack_start(ready_but, Gtk::PACK_EXPAND_WIDGET, 20);


   // Gamestatus Font, etc
   Pango::AttrList pango_attr("<span size=\"xx-large\" weight=\"ultrabold\">Halted Running Stopped 0123456789:</span>");
   game_status_label.set_attributes(pango_attr);
   time_label.set_attributes(pango_attr);
   Pango::AttrList pango_attr2("<span size=\"80000\" weight=\"ultrabold\">0123456789:., </span>");
   yellow_goal.set_attributes(pango_attr2);
   vs.set_attributes(pango_attr2);
   blue_goal.set_attributes(pango_attr2);

   goal_hbox.pack_start(yellow_goal, Gtk::PACK_EXPAND_WIDGET, 10);
   goal_hbox.pack_start(vs, Gtk::PACK_EXPAND_WIDGET, 10);
   goal_hbox.pack_start(blue_goal, Gtk::PACK_EXPAND_WIDGET, 10);
   goal_vbox.add ( goal_hbox );

   teamname_yellow.modify_base(Gtk::STATE_NORMAL, Gdk::Color("lightyellow"));
   teamname_yellow.set_has_frame(false);
   teamname_yellow.set_alignment(Gtk::ALIGN_CENTER);
   teamname_blue.modify_base(Gtk::STATE_NORMAL, Gdk::Color("lightblue"));
   teamname_blue.set_has_frame(false);
   teamname_blue.set_alignment(Gtk::ALIGN_CENTER);
   teamname_hbox.pack_start(teamname_yellow, Gtk::PACK_EXPAND_WIDGET, 10);
   teamname_hbox.pack_start(teamname_blue,   Gtk::PACK_EXPAND_WIDGET, 10);

   goal_vbox.add ( teamname_hbox );
   goal_frame.add ( goal_vbox );

   game_control_box.set_homogeneous(true);
   game_control_box.pack_start(cancel_but, Gtk::PACK_EXPAND_WIDGET, 10,10);

   yellow_goal_box.pack_start(yellow_goal_but, Gtk::PACK_EXPAND_WIDGET);
   yellow_goal_box.pack_start(yellow_subgoal_but, Gtk::PACK_EXPAND_WIDGET);
   blue_goal_box.pack_start(blue_goal_but, Gtk::PACK_EXPAND_WIDGET);
   blue_goal_box.pack_start(blue_subgoal_but, Gtk::PACK_EXPAND_WIDGET);

   game_control_box.pack_start(yellow_goal_box, Gtk::PACK_EXPAND_WIDGET, 10,10);
   game_control_box.pack_start(blue_goal_box, Gtk::PACK_EXPAND_WIDGET, 10,10);

   game_status_vbox.pack_start( time_label, Gtk::PACK_EXPAND_WIDGET);
   game_status_vbox.pack_start( game_status_label, Gtk::PACK_EXPAND_WIDGET);
   game_status_hbox.pack_start( game_status_vbox, Gtk::PACK_EXPAND_WIDGET);
   game_status_hbox.pack_start( game_control_box, Gtk::PACK_EXPAND_WIDGET);
   game_control_frame.add( game_status_hbox );

   big_vbox.pack_start( menu_bar, Gtk::PACK_SHRINK );
   big_vbox.pack_start( halt_stop_hbox, Gtk::PACK_EXPAND_WIDGET, 10 );
   big_vbox.pack_start( start_ready_hbox, Gtk::PACK_EXPAND_WIDGET, 10 );
   big_vbox.pack_start( goal_frame, Gtk::PACK_EXPAND_WIDGET, 10 );   
   //big_vbox.pack_start( game_status_hbox, Gtk::PACK_SHRINK, 10 );
   big_vbox.pack_start( game_control_frame, Gtk::PACK_SHRINK, 10 );
   big_vbox.pack_start( team_hbox, Gtk::PACK_EXPAND_WIDGET, 10);
   
   add ( big_vbox );
   show_all();
}

Refereemm_Main_Window::~Refereemm_Main_Window()
{
}

// signale
void Refereemm_Main_Window::on_exit_clicked()
{
   exit(0);
}

void Refereemm_Main_Window::on_load_config()
{
}

void Refereemm_Main_Window::on_toggle_enable_commands()
{
   gamecontrol.toggleEnable();   
}

void Refereemm_Main_Window::on_start_button()
{
   gamecontrol.setStart();   
}

void Refereemm_Main_Window::on_stop_button()
{
   gamecontrol.setStop();
}

void Refereemm_Main_Window::on_halt()
{
   gamecontrol.setHalt();
}

void Refereemm_Main_Window::on_cancel()
{
   gamecontrol.setCancel();
}

void Refereemm_Main_Window::on_ready()
{
   gamecontrol.setReady();
}

void Refereemm_Main_Window::on_yellow_goal()
{
   gamecontrol.goalScored(Yellow);
}

void Refereemm_Main_Window::on_blue_goal()
{
   gamecontrol.goalScored(Blue);
}

void Refereemm_Main_Window::on_yellow_subgoal()
{
   gamecontrol.removeGoal(Yellow);
}

void Refereemm_Main_Window::on_blue_subgoal()
{
   gamecontrol.removeGoal(Blue);
}

void Refereemm_Main_Window::on_yellow_kickoff()
{
   gamecontrol.setKickoff(Yellow);
}

void Refereemm_Main_Window::on_yellow_freekick()
{
   gamecontrol.setDirect(Yellow);
}

void Refereemm_Main_Window::on_yellow_penalty()
{
   gamecontrol.setPenalty(Yellow);
}

void Refereemm_Main_Window::on_yellow_indirect_freekick()
{
   gamecontrol.setIndirect(Yellow);
}

void Refereemm_Main_Window::on_yellow_yellowcard()
{
   gamecontrol.awardYellowCard(Yellow);
}

void Refereemm_Main_Window::on_yellow_redcard()
{
   gamecontrol.awardRedCard(Yellow);
}

void Refereemm_Main_Window::on_yellow_timeout_start()
{
   gamecontrol.beginTimeout(Yellow);
}

void Refereemm_Main_Window::on_yellow_timeout_stop()
{
  //   gamecontrol.stopTimeout();
}

void Refereemm_Main_Window::on_blue_kickoff()
{
   gamecontrol.setKickoff(Blue);
}

void Refereemm_Main_Window::on_blue_freekick()
{
   gamecontrol.setDirect(Blue);
}

void Refereemm_Main_Window::on_blue_penalty()
{
   gamecontrol.setPenalty(Blue);
}

void Refereemm_Main_Window::on_blue_indirect_freekick()
{
   gamecontrol.setIndirect(Blue);
}


void Refereemm_Main_Window::on_blue_yellowcard()
{
   gamecontrol.awardYellowCard(Blue);
}

void Refereemm_Main_Window::on_blue_redcard()
{
   gamecontrol.awardRedCard(Blue);
}

void Refereemm_Main_Window::on_blue_timeout_start()
{
  gamecontrol.beginTimeout(Blue);
}

void Refereemm_Main_Window::on_blue_timeout_stop()
{
  //  gamecontrol.stopTimeout();
}

void Refereemm_Main_Window::on_teamname_yellow()
{
    std::string name(teamname_yellow.get_text());
    
    // no ',' allowed in teamname
    size_t i;
    while (std::string::npos != (i = name.find(',')))
    {
        name.replace(i, 1, " ");
    }
    
    gamecontrol.setTeamName(Yellow, name);
}

void Refereemm_Main_Window::on_teamname_blue()
{
    std::string name(teamname_blue.get_text());
    
    // no ',' allowed in teamname
    size_t i;
    while (std::string::npos != (i = name.find(',')))
    {
        name.replace(i, 1, " ");
    }
    
    gamecontrol.setTeamName(Blue, name);
}

void Refereemm_Main_Window::set_active_widgets(const EnableState& es)
{
   halt_but.set_sensitive(es.halt);
   ready_but.set_sensitive(es.ready);
   stop_but.set_sensitive(es.stop);
   start_but.set_sensitive(es.start);

   cancel_but.set_sensitive(es.cancel);

   // Yellow
   yellow_kickoff_but.set_sensitive(es.kickoff[Yellow]);
   yellow_goal_but.set_sensitive(es.goal[Yellow]);   
   yellow_subgoal_but.set_sensitive(es.subgoal[Yellow]);   
   yellow_freekick_but.set_sensitive(es.direct[Yellow]);
   yellow_indirect_freekick_but.set_sensitive(es.indirect[Yellow]);
   yellow_penalty_but.set_sensitive(es.penalty[Yellow]);
   yellow_timeout_start_but.set_sensitive(es.timeout[Yellow]);
   yellow_yellowcard_but.set_sensitive(es.cards);
   yellow_redcard_but.set_sensitive(es.cards);
   
   //Blue Team
   blue_kickoff_but.set_sensitive(es.kickoff[Blue]);
   blue_goal_but.set_sensitive(es.goal[Blue]);   
   blue_subgoal_but.set_sensitive(es.subgoal[Blue]);
   blue_freekick_but.set_sensitive(es.direct[Blue]);
   blue_indirect_freekick_but.set_sensitive(es.indirect[Blue]);
   blue_penalty_but.set_sensitive(es.penalty[Blue]);
   blue_timeout_start_but.set_sensitive(es.timeout[Blue]);
   blue_yellowcard_but.set_sensitive(es.cards);
   blue_redcard_but.set_sensitive(es.cards);
}

void Refereemm_Main_Window::idle()
{
   // First get new time step
   gamecontrol.stepTime();
   char str[1024];
   GameInfo gi = gamecontrol.getGameInfo();

//   std::cout << gi.data << std::endl;
   
   // Update the Gui
   game_status_label.set_text( gi.getStateString() );

   teamname_blue.set_text( gi.data.teamnames[Blue] );
   teamname_yellow.set_text( gi.data.teamnames[Yellow] );
   

   if (gi.data.stage == PENALTY_SHOOTOUT) {
      // Needs extra handling.
      sprintf(str, "%2i", gi.data.goals[Yellow]);
      yellow_goal.set_text( str );
      sprintf(str, "%2i", gi.data.goals[Blue]);
      blue_goal.set_text( str );

      sprintf(str, "Penalties:\n %i - %i\n", 
	      gi.data.penaltygoals[Yellow], gi.data.penaltygoals[Blue]);
      time_label.set_text( str );
   } else {
      sprintf(str, "%2i", gi.data.goals[Yellow]);
      yellow_goal.set_text( str );
      sprintf(str, "%2i", gi.data.goals[Blue]);
      blue_goal.set_text( str );

      sprintf(str, "%s\n%2i:%04.1f\n%2i:%04.1f", 
              gi.getStageString(),
              DISP_MIN(gi.timeTaken()),
              DISP_SEC(gi.timeTaken()),
              DISP_MIN(gi.timeRemaining()), 
              DISP_SEC(gi.timeRemaining()));
      time_label.set_text( str );
   }

   sprintf(str, "%i", gi.nrTimeouts(Yellow) );
   yellow_timeouts_left_text.set_text( str );
   sprintf(str, "%i", gi.nrTimeouts(Blue) );
   blue_timeouts_left_text.set_text( str );
   
   sprintf(str, "%2i:%04.1f",
	   DISP_MIN(gi.timeoutRemaining(Yellow)), 
	   DISP_SEC(gi.timeoutRemaining(Yellow)));
   yellow_timeout_time_text.set_text(str);

   if (gi.penaltyTimeRemaining(Yellow) > 0) {
     sprintf(str, "%2i:%04.1f",
	   DISP_MIN(gi.penaltyTimeRemaining(Yellow)), 
	   DISP_SEC(gi.penaltyTimeRemaining(Yellow)));
     yellow_yellowcard_but.set_label(str);
     yellow_yellowcard_but.set_sensitive(false);
   }
   else {
     yellow_yellowcard_but.set_label("Yellow Card");
     yellow_yellowcard_but.set_sensitive(true);
   }
       
   sprintf(str, "%2i:%04.1f",
	   DISP_MIN(gi.timeoutRemaining(Blue)),
	   DISP_SEC(gi.timeoutRemaining(Blue)));
   blue_timeout_time_text.set_text(str);

   if (gi.penaltyTimeRemaining(Blue) > 0) {
     sprintf(str, "%2i:%04.1f",
	   DISP_MIN(gi.penaltyTimeRemaining(Blue)),
	   DISP_SEC(gi.penaltyTimeRemaining(Blue)));
     blue_yellowcard_but.set_label(str);
     blue_yellowcard_but.set_sensitive(false);
   }
   else {
     blue_yellowcard_but.set_label("Yellow Card");
     blue_yellowcard_but.set_sensitive(true);
   }
   
   set_active_widgets(gamecontrol.getEnableState());

   #ifdef WIN32
   Sleep(50);
   #else
   usleep(50000);
   #endif
}



int main(int argc, char* argv[])
{
   char c;
#ifndef WIN32
   //std::string configfile = "/etc/sslrefbox/referee.conf";
   std::string configfile = "referee.conf";
#else
   std::string configfile = "referee.conf";
#endif
   std::string logfile = "gamecontrol";
   bool restart = false;

   // Let Gtk take it's arguments
   Gtk::Main kit(argc, argv);
   
   while ((c = getopt(argc, argv, "hC:l:r")) != EOF) {
      switch (c) {
      case 'C': configfile = optarg; break;
      case 'r': restart = true; break;
      case 'l': logfile = optarg; break;
      case 'h':
      default:
        fprintf(stderr, "\nSmallSize Referee Program\n");
        fprintf(stderr, "(c) RoboCup Federation, 2003\n");
        fprintf(stderr, "\nUSAGE\n");
        fprintf(stderr, "referee -[hC]\n");
        fprintf(stderr, "\t-h\t\tthis help message\n");
        fprintf(stderr, "\t-C <file>\tUse config file <file>\n");
        fprintf(stderr, "\t-l <file>\tUse logfile prefix <file>\n");
        fprintf(stderr, "\t-r \trestore previous game\n");
        exit(0);
      }
   }

   GameControl gamecontrol;

   // load everything
   if (!gamecontrol.init(configfile.c_str(), logfile.c_str(), restart)) {
      fprintf(stderr, "ERROR: Cannot intialize game controller\n");
      return (1);
   }

   // setup the GUI
   Refereemm_Main_Window* main;
   main = new Refereemm_Main_Window(gamecontrol);
   kit.run(*main);
   delete main;
   gamecontrol.close();
   return 0;
}

