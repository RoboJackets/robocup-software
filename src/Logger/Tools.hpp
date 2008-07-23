#ifndef TOOLS_HPP_
#define TOOLS_HPP_

#include <QWidget>
#include <QMouseEvent>
#include <QTimer>

#include <Team.h>

#include <Packet/PacketSender.hpp>
#include <Packet/MotionCmd.hpp>
#include <Packet/SkillCmd.hpp>
#include <Packet/VisionData.hpp>

class QPushButton;

class Tools : public QWidget
{
	Q_OBJECT;

	public:
		Tools(Team t, Packet::VisionData& vd, QWidget* parent = 0);
		~Tools();

	public Q_SLOTS:
		void newPosition(float x, float y, float wx, float wy, QMouseEvent me);
		
		void changeRobot(int rid);
		
		void gotoBall();
		void handleBall();
		void noSkill();

    protected Q_SLOTS:
        void send();
        void stopSending();
	
    private:
        void sendSkill();
		
        /** currently active robot */
		unsigned int _rid;

		/** sender for tool commands */
		Packet::PacketSender _sender;

		//motion stuff
		QPushButton* _gotoPos;
		QPushButton* _facePos;
		QPushButton* _none;
		
		//skills
		QPushButton* _noSkill;
		QPushButton* _gotoBall;
		QPushButton* _handleBall;
		
		Packet::SkillCmd::Skill _lastSkill;

		//current angle and position points
		Geometry::Point2d _pos[5];
		Geometry::Point2d _face[5];
		
		Packet::VisionData& _vision;
        
        QTimer _timer;
};

#endif /*TOOLS_HPP_*/
