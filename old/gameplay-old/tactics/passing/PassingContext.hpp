
#pragma once

#include <gameplay/tactics/passing/DumbReceive.hpp>
#include <gameplay/tactics/passing/StablePass.hpp>
#include <gameplay/PreventDoubleTouch.hpp>
#include <Geometry2d/Segment.hpp>

namespace Gameplay {

	class PassingContext {
	public:
		PassingContext(GameplayModule *gameplay, StablePass *passer);

		static void createConfiguration(Configuration *cfg);


		bool run();	//	note: currently doesn't do any robot assignment - that's up to you
		bool done();


		void setReceivePointForReceiver(DumbReceive *receiver, Geometry2d::Point &rcvPt);
		void chooseReceivePointForReceiverAlongSegment(DumbReceive *rcvr, Geometry2d::Segment &segment);	//	note: updates the score for the receiver also

		void setReceiverScore(DumbReceive *rcvr, float score);
		void updateReceiverScoresBasedOnChannelWidth();

		
		DumbReceive *currentBestReceiver(float *out_score = NULL);
		DumbReceive *currentChosenReceiver();


		void addReceiver(DumbReceive *rcvr);
		void removeReceiver(DumbReceive *rcvr);

	protected:
		void setPasser(StablePass *passer);	//	TODO: rm

		void chooseReceiver(DumbReceive *rcvr = NULL);	//	if you pass NULL or leave @rcvr blank, it will choose the best receiver based on scores
		void _chooseReceiver(DumbReceive *rcvr);

		void _updatePairing();
		void _unkickAllReceivers();
		void _setAllReceiverDribblers(uint8_t dSpeed = 60);


	private:
		std::vector<DumbReceive *> _receivers;
		std::map<DumbReceive *, float> _scoresByReceiver;

		PreventDoubleTouch _pdt;
		StablePass *_passer;
		DumbReceive *_chosenReceiver;

		bool _passDone;

		GameplayModule *_gameplay;

		static ConfigDouble *_receiverChoiceHysteresis;
	};

}
