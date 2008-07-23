#include "Playbook.hpp"
#include "Play.hpp"
#include "Named_Matrix.hpp"
#include "tactics/Tactics.hpp"

#include "tactics/Goalie.hpp"

#include <antlr/ANTLRException.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace antlr;
using namespace boost;
using namespace boost::filesystem;
using namespace Packet;

Playbook::Playbook()
{
    _goalie = 0;
    _current_play = 0;
    _need_start = false;
    _logged_end = false;
    _available_robots = 0;
}

void Playbook::load_dir(const string &path)
{
    directory_iterator end;
    for (directory_iterator i(path); i != end; ++i)
    {
        file_status status = i->status();
        if (is_regular(status))
        {
            printf("loading %s\n", i->path().string().c_str());
            load(i->path().string());
        } else if (is_directory(status) && (*i->path().leaf().begin()) != '.')
        {
            load_dir(i->path().string());
        }
    }
}

void Playbook::load(const std::string &path)
{
    try
    {
        Play *play = new Play(path.c_str());
        _plays.push_back(play);
    } catch (ANTLRException &e)
    {
        printf("Error: %s\n", e.toString().c_str());
    }
}

bool Playbook::setup()
{
    if (_goalie)
    {
#if 0
        //FIXME - The goalie can only be changed under certain circumstances.
        if (_goalie->assigned() && !_goalie->robot()->visible())
        {
            // Lost the goalie
            _goalie->robot(0);
        }
#endif
        
        // Assign a goalie if we don't have one.
        if (!_goalie->assigned())
        {
            // Try to assign a robot to be the goalie
            Robot *robot = _goalie->select_robot();
            if (robot)
            {
                // We have a new goalie
                printf("Robot %d is the goalie\n", robot->id());
                _goalie->robot(robot);
                robot->goalie(true);
            }
        }
    }
    
    // Select a new play if we need one.
    if (!_current_play || _current_play->done())
    {
        // Log why the play ended, with statistics.
        if (_current_play && !_logged_end)
        {
            // Don't do this again until this or another play is started.
            _logged_end = true;
            
            printf("Play \"%s\" ended: ", _current_play->name().c_str());
            switch (_current_play->status())
            {
                case Play::Timeout:
                    printf("timeout %d/%d\n", _current_play->num_timeouts(), _current_play->num_started());
                    break;
                
                case Play::Terminated:
                    printf("terminated %d/%d\n", _current_play->num_terminated(), _current_play->num_started());
                    break;
                
                case Play::Completed:
                    printf("completed %d/%d\n", _current_play->num_completed(), _current_play->num_started());
                    break;
                
                case Play::Aborted:
                    printf("aborted %d/%d\n", _current_play->num_aborted(), _current_play->num_started());
                    break;
                
                default:
                    printf("BAD STATUS\n");
                    break;
            }
        }
        
        // Switch to a new play if it is different from the current play or
        // if it is the same and the current play wants to be restarted.
        Play *next = select_play();
        if (next && (next != _current_play || (_current_play && _current_play->restart)))
        {
            // Call stop() here to update statistics.
            if (_current_play)
            {
                _current_play->stop();
            }
            
            _current_play = next;
            _need_start = true;
            
            printf("Switching to play \"%s\"\n", _current_play->name().c_str());
            
            _logged_end = false;
        }
    }
    
    return _need_start;
}

void Playbook::run()
{
    // Clear the motion command
    skill_packet = SkillCmd();
    
    // Start the play before running the goalie in case the goalie
    // needs to work with other robots.
    if (_current_play)
    {
        if (_need_start)
        {
            _need_start = false;
            _current_play->start();
        }
    }
    
    for(int i=0 ; i<5 ; ++i)
    {
    	Robot::self[i].free(true);
    }
            
    // Run the goalie behavior.
    if (_goalie && _goalie->assigned())
    {
        _goalie->run();
    }
    
    // Run the current play.
    if (_current_play)
    {
        _current_play->run();
    }
}

void Playbook::abort()
{
    if (_current_play)
    {
        if (!_need_start)
        {
            _current_play->stop();
        }
        _current_play = 0;
    }
    _need_start = false;
}

// Returns true if p1 is better than p2.
bool Playbook::compare_plays(Play *p1, Play *p2)
{
    // Prefer roles for which we have enough robots.
    int n1 = p1->num_roles();
    int n2 = p2->num_roles();
    if (_available_robots < n1 && _available_robots >= n2)
    {
        // p1 needs too many robots.
        return false;
    } else if (_available_robots >= n1 && _available_robots < n2)
    {
        // p2 needs too many robots.
        return true;
    }
    
    // Both plays have enough robots.
    
    if (p1->preference_value() > p2->preference_value())
    {
        return true;
    } else if (p1->preference_value() < p2->preference_value())
    {
        return false;
    } else {
        //FIXME - Should this move into preferences too?
        // Prefer untested plays.
        if (p1->num_started() == 0 && p2->num_started() > 0)
        {
            // p1 hasn't been run but p2 has.
            return true;
        }
        
        if (p2->num_started() == 0)
        {
            // p2 hasn't been run but p1 has.
            return false;
        }
    }
    
    // Equally good
    return false;
}

void print_plays(const char *label, const vector<Play *> &plays)
{
    printf("%s:\n", label);
    BOOST_FOREACH(Play *play, plays)
    {
        printf("    %p: %s\n", play, play->name().c_str());
    }
}

Play *Playbook::select_play()
{
    // Count how many robots are available.
    _available_robots = 0;
    for (int i = 0; i < 5; ++i)
    {
        if (Robot::self[i].visible())
        {
            _available_robots++;
        }
    }
    
    // Subtract one for the goalie.
    if (_available_robots && _goalie && _goalie->assigned())
    {
        --_available_robots;
    }

    // Can't select a play with no robots.
    if (_available_robots == 0)
    {
        return 0;
    }

    // Build a vector of all plays that can work in the current situation.
    // This must be a vector for sort.
    vector<Play *> applicable;
    BOOST_FOREACH(Play *play, _plays)
    {
        if (play->require.all())
        {
            applicable.push_back(play);
            play->update_preference();
        }
    }
    
    // NOTE: This is unintuitive, but correct based on how I have defined things:
    //       If you have two equally good non-restarting plays, they will appear
    //       to alternate even though plays should be chosen randomly.
    //       This happens because the first play finishes and a new play is
    //       randomly selected each frame.  If the first play is reselected,
    //       nothing happens because it is current and non-restarting.
    //       The reselection continues each frame until the second play is selected.
    //       After the second play finishes reselection happens again until
    //       the first play is selected.
    //
    //       The notable result is that a non-restarting play can run for a
    //       random number of frames after it has finished if it is just as good
    //       as other plays.
    
    // Randomize first, so plays with the same score will be chosen randomly.
    random_shuffle(applicable.begin(), applicable.end());
//    print_plays("Random", applicable);
    
    // Sort by score, with untested plays at the top.
    stable_sort(applicable.begin(), applicable.end(), bind(&Playbook::compare_plays, this, _1, _2));
//    print_plays("Sorted", applicable);
    
    if (!applicable.empty())
    {
        return applicable.front();
    } else {
        return 0;
    }
}

void Playbook::goalie(Tactics::Base *goalie)
{
    Robot *robot = 0;
    if (_goalie)
    {
        robot = _goalie->robot();
        _goalie->robot(0);
    }
    
    _goalie = goalie;
    
    // If we are changing goalie behaviors, keep the same robot.
    if (_goalie && robot)
    {
        // This will start the goalie.
        _goalie->robot(robot);
    }
}
