#ifndef SOCCSIMWINDOW_HPP_
#define SOCCSIMWINDOW_HPP_

#include <QMainWindow>
#include <ui_SimulatorWindow.h>

#include "Viewer.hpp"
#include "Physics/Env.hpp"
#include "Vision.hpp"

class SoccSimWindow : public QMainWindow
{
	Q_OBJECT;
	
	public:
		SoccSimWindow();
		~SoccSimWindow();
		
	protected Q_SLOTS:
		void on_blueToggle_toggled(bool b);
		void on_yellowToggle_toggled(bool b);
		
		void on_b0_toggled(bool b) { toggleRobot(Blue, 0, b); }
		void on_b1_toggled(bool b) { toggleRobot(Blue, 1, b); }
		void on_b2_toggled(bool b) { toggleRobot(Blue, 2, b); }
		void on_b3_toggled(bool b) { toggleRobot(Blue, 3, b); }
		void on_b4_toggled(bool b) { toggleRobot(Blue, 4, b); }
		
		void on_y0_toggled(bool b) { toggleRobot(Yellow, 0, b); }
		void on_y1_toggled(bool b) { toggleRobot(Yellow, 1, b); }
		void on_y2_toggled(bool b) { toggleRobot(Yellow, 2, b); }
		void on_y3_toggled(bool b) { toggleRobot(Yellow, 3, b); }
		void on_y4_toggled(bool b) { toggleRobot(Yellow, 4, b); }
		
		void on_ball_toggled(bool b);
		
		void on_actionError_triggered(bool c);

	protected:
		void toggleRobot(Team t, unsigned int id, bool active);
		
	/// members ///
	protected:
		/** Simulated evironment */
		Physics::Env _env;
		
		/** vision data sender */
		Vision _vision;
		
		Ui::SimulatorWindow _ui;
		
		Viewer viewer;
};

#endif /*SOCCSIMWINDOW_HPP_*/
