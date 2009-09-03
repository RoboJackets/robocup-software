#pragma once

#include "Condition.hpp"
#include "Opponent.hpp"

#include <string>
#include <list>
#include <QTime>

namespace Gameplay
{
    class Play;
    class Role;
    class Parameter;
    class GameplayModule;

    class Param_Name_Ref
    {
        public:
            Param_Name_Ref(Play *play, Parameter *param, const std::string &name)
            {
                this->play = play;
                this->param = param;
                this->name = name;
            }
            
            Play *play;
            Parameter *param;
            std::string name;
    };

    class Play
    {
    public:
        class Preference
        {
        public:
            virtual ~Preference() {};
            virtual float value() const = 0;
        };
        
        class Predicate_Preference: public Preference
        {
        public:
            Predicate_Preference(Predicate *p, float w)
            {
                predicate = p;
                weight = w;
            }
            
            virtual float value() const;
            
        protected:
            Predicate *predicate;
            float weight;
        };
        
        class Play_Preference: public Preference
        {
        public:
            Play_Preference(Play *play)
            {
                _play = play;
            }
        
        protected:
            Play *_play;
        };
        
        class Completed_Preference: public Play_Preference
        {
        public:
            Completed_Preference(Play *play): Play_Preference(play) {}
            
            virtual float value() const;
        };
        
        // Reason the play ended (only valid if done() == true).
        typedef enum
        {
            Running = 0,        // Play has not ended.
            Timeout,            // Timeout exceeded.
            Terminated,         // At least one termination condition was met.
            Completed,          // All roles have finished.
            Aborted             // Play aborted due to change in game state.
        } Status;
        
        Play(GameplayModule *gameplay);
        Play(GameplayModule *gameplay, const char *filename);
        ~Play();
        
        GameplayModule *gameplay() const
        {
            return _gameplay;
        }
        
        // Sets default preferences for this play if none are present.
        void defaultPreferences();
        
        // Starts all roles.
        void start();
        
        // Runs all current tactics and advances the roles if permitted.
        //
        // This also sets _status.  Playbook::run() is responsible for
        // starting and stopping plays in response to play status changes.
        //
        // This also updates statistics if the play has just finished.
        void run();
        
        // Stops all roles.
        void stop();
        
        // Returns true if another play should be selected.
        bool done() const { return _status != Running; }
        
        Status status() const { return _status; }
        
        const std::string &name() const { return _name; }
        void name(const std::string &value)
        {
            _name = value;
        }
        
        Role *role(const std::string &name);
        int num_roles() const { return _roles.size(); }
        
        // Statistics
        int numStarted() const { return _numStarted; }
        int numTimeouts() const { return _numTimeouts; }
        int numTerminated() const { return _numTerminated; }
        int numCompleted() const { return _numCompleted; }
        int numAborted() const { return _numAborted; }
        
        // Resets statistics.
        void resetStats();
        
        // Updates _preference_value based on PREFER terms.
        void updatePreference();
        
        // Returns the most recently calculated preference value.
        float preferenceValue() const { return _preferenceValue; }
        
        // Adds an item to _preferences by name.
        bool addPreference(const std::string &name, float weight);
        
        void opponent(const std::string &id, const std::string &desc);
        
        Condition require;
        Condition terminate;
        std::list<Preference *> preferences;
        
        // How long the play is allowed to run, in seconds.
        // If negative, there is no limit.
        float timeout;
        
        // This flag affects what happens when the play finishes and is selected again
        // by the Playbook.
        //
        // If true, the play is restarted (start() is called, then stop()).  This is the default.
        // If false, the play continues without restarting.
        bool restart;
        
        // Parameters which take robot names.
        // These must be resolved when the play is started so opponents can be selected.
        std::list<Param_Name_Ref> nameParams;
        
    protected:
        friend class Role;
        
        GameplayModule *_gameplay;
        std::string _name;
        std::list<Role *> _roles;
        std::list<Opponent *> _opponents;
        Status _status;
        
        QTime _startTime;
        
        // True after stats have been updated for this time the play was run.
        bool _statsDone;
        
        // Number of times this play has been started.
        int _numStarted;
        
        // Number of times this play has timed out.
        int _numTimeouts;
        
        // Number of times this play has been terminated by DONE conditions.
        int _numTerminated;
        
        // Number of times this play has completed all tactics.
        int _numCompleted;
        
        // Number of times this play has been aborted.
        int _numAborted;
        
        // Total value of PREFER terms.
        // Updated by update_preference().
        float _preferenceValue;
        
        // Common initialization for constructors.
        void init(GameplayModule *gameplay);
    };
}
