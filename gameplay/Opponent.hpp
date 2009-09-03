#pragma once

#include <string>

namespace Gameplay
{
    class Robot;
    class GameplayModule;
    
    class Opponent
    {
    public:
        Opponent(GameplayModule *gameplay, const std::string &name);
        virtual ~Opponent();
        
        void select();
        
        const std::string &name() const { return _name; }
        Robot *robot() const { return _robot; }

    protected:
        // Lower scores are better.
        virtual float score(Robot *robot) const = 0;
        
        GameplayModule *_gameplay;
        std::string _name;
        Robot *_robot;
    };

    class Opponent_Near_Ball: public Opponent
    {
        public:
            Opponent_Near_Ball(GameplayModule *gameplay, const std::string &name): Opponent(gameplay, name) {}
        
        protected:
            virtual float score(Robot *robot) const;
    };

    class Opponent_Near_Goal: public Opponent
    {
        public:
            Opponent_Near_Goal(GameplayModule *gameplay, const std::string &name): Opponent(gameplay, name) {}
        
        protected:
            virtual float score(Robot *robot) const;
    };
}
