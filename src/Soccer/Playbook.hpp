#ifndef _PLAYBOOK_H_
#define _PLAYBOOK_H_

#include <string>
#include <list>

class Play;
class Goalie;

namespace Tactics
{
    class Base;
}

class Playbook
{
public:
    Playbook();
    
    // Loads all .play files from the given directory.
    void load_dir(const std::string &path);
    
    // Loads a single play from a file.
    void load(const std::string &path);

    // Selects a goalie and a play if necessary.
    // Returns true if a new play has been selected.
    bool setup();
    
    // Runs the goalie and the current play.
    void run();
    
    // Aborts the current play.
    // A new play will be selected when run() is called.
    void abort();

    // Sets the goalie tactic.
    // The goalie always has top priority when assigning robots.
    // If any robots are available, the goalie will be assigned one.
    void goalie(Tactics::Base *goalie);

    // Selects a new play.
    Play *select_play();

    Play *current_play() const { return _current_play; }
    const std::list<Play *> &plays() const { return _plays; }

protected:
    // Used for sorting plays.
    bool compare_plays(Play *p1, Play *p2);
    int _available_robots;
    
    Tactics::Base *_goalie;
    Play *_current_play;
    
    // True if the current play has just been selected and needs to be started in run().
    bool _need_start;
    
    std::list<Play *> _plays;
    
    // True when the reason the current play ended has been printed.
    bool _logged_end;
};

#endif // _PLAYBOOK_H_
