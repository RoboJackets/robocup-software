
import main




class SituationalPlaySelector:


    scoreBonus = 100

    situations = {
            'kickoff' : 0,
            'indirect_kick' : 0,
            'direct_kick' : 0,
            'defend_restart_offensive' : 0,
            'defend_restart_defensive' : 0,
            'clear' : 0,
            'defend_clear' : 0,
            'defend_goal' : 0,
            'attack_goal' : 0,
            'offensive_scramble' : 0,
            'defensive_scramble' : 0,
            'save_ball' : 0,
            'save_shot' : 0,
            'offensive_pile_up' : 0,
            'defensive_pile_up': 0}
        
    gameState = main.game_state()

    @staticmethod
    def updateAnalysis():
        pass 

    @staticmethod
    def ballPossessionUpdate():
        pass

    @staticmethod
    def getBonus():
        pass













