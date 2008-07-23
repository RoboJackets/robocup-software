header
{
#include "Play.hpp"
#include "tactics/parameters/Parameter.hpp"
}

options
{
    language = "Cpp";
}

////////
// Lexer

class Play_Lexer extends Lexer;

options
{
    caseSensitiveLiterals = false;
}

// End-of-line
protected EOL: ('\n' { newline(); } | '\r');

// Whitespace and comments are ignored
COMMENT:    "//" (~('\n'|'\r'))* EOL
    { $setType(antlr::Token::SKIP); };

WHITESPACE: (' ' | '\t' | EOL)
    { $setType(antlr::Token::SKIP); };

ROBOT_ID:    '@' ('0'..'9'|'a'..'z'|'A'..'Z'|'_')+;

NUMBER:     ('-')?
            (('0'..'9')+ ('.' ('0'..'9')+)? | '.' ('0'..'9')+)
            ('e' ('-')? ('0'..'9')+)?;

PUNCT:      (','|'('|')'|'='|'!'|':');

IDENTIFIER options { testLiterals = true; }:
    ('a'..'z'|'A'..'Z'|'_')('0'..'9'|'a'..'z'|'A'..'Z'|'_')*
    ;

STRING: '"' (~('"'|'\\')|"\\\"")* '"';

////////
// Parser

{
#include "Named_Matrix.hpp"
#include "Condition.hpp"
#include "Predicate.hpp"
#include "Role.hpp"
#include "tactics/Tactics.hpp"

#include <boost/format.hpp>

using namespace std;
using namespace boost;

void Play_Parser::reportError(const antlr::RecognitionException &e)
{
    // By default, this is just printed.  Need to re-throw the exception
    // to make sure parsing fails.
    throw e;
}

}

class Play_Parser extends Parser;

options
{
    k = 2;
}

{
public:
    virtual void reportError(const antlr::RecognitionException &e);

protected:
    Play *_play;
}

play_file[Play *play]:
            { _play = play; }
            (decl)+ EOF;

decl:       
            name |
            restart |
            require |
            prefer |
            done |
            timeout |
            opponent |
            role;

name:
            "NAME" name:STRING
            { const string &str = name->getText();
              _play->name(str.substr(1, str.size() - 2));
            };
            
restart:
            "NO_RESTART" { _play->restart = false; };

require:
            "REQUIRE" condition[_play->require];

prefer:
            "PREFER" preference ("," preference)*;

preference:
            { float weight = 1; }
            name:IDENTIFIER (":" weight=number)?
            { if (!_play->add_preference(name->getText(), weight))
              {
                throw runtime_error(str(format("No such predicate \"%1%\"") % name->getText()));
              }
            };

done:
            "DONE" condition[_play->terminate];

condition[Condition &cond]:
            predicate[cond] ("," predicate[cond])*;

predicate[Condition &cond]:
            { bool invert = false; }
            ("!" { invert = true; })? name:IDENTIFIER
            { Predicate *pred = Predicate::find(name->getText());
              if (!pred)
              {
                throw runtime_error(str(format("No such predicate \"%1%\"") % name->getText()));
              }
              cond.add(pred, invert);
            };

timeout:
            { float t; }
            "TIMEOUT" t=number
            { _play->timeout = t; };

opponent:
            { std::string id; }
            "OPPONENT" id=robot_name desc:IDENTIFIER
            { _play->opponent(id, desc->getText()); }
            ;

role:
            { std::string id; }
            "ROLE" id=robot_name
            { Role *role = new Role(_play, id); }
            (tactic[role])*;

tactic[Role *role]:
            name:IDENTIFIER
                { Tactics::Base *tactic = Tactics::Factory::create(name->getText(), role);
                  if (!tactic)
                  {
                    throw runtime_error(str(format("Unknown tactic \"%1%\"") % name->getText()));
                  }
                }
            (parameter[tactic])*;

parameter[Tactics::Base *tactic]:
            name:IDENTIFIER
                { Tactics::Parameter *param = tactic->parameter(name->getText());
                  if (!param)
                  {
                    throw runtime_error(str(format("Tactic \"%1%\" has no parameter \"%2%\"") % tactic->name() % name->getText()));
                  }
                }
            "=" expression[tactic, param];

expression[Tactics::Base *tactic, Tactics::Parameter *param]:
            point[param]
            
            | { float v; } v=number
            { param->set(v); }
            
            | { std::string id; } id=robot_name
            { _play->name_params.push_back(Param_Name_Ref(_play, param, id)); }
            
            | s:IDENTIFIER
            { param->set(s->getText()); };

point [Tactics::Parameter *param]:
            { float x, y;
              Named_Matrix *matrix = 0;
            }
            (id:IDENTIFIER ":"
            { matrix = Named_Matrix::find(id->getText());
              if (!matrix)
              {
                throw runtime_error(str(format("Unknown matrix \"%1%\"") % id->getText()));
              }
            } )?
            "(" x=number "," y=number ")"
            { Geometry::Point2d point(x, y);
              if (matrix)
              {
                param->set(matrix, point);
              } else {
                param->set(point);
              }
            };

number returns [float v = 0]:
            str:NUMBER
            { v = atof(str->getText().c_str()); };

robot_name returns [std::string name]:
            str:ROBOT_ID
            { name = str->getText().substr(1); };
