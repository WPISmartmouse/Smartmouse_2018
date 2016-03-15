#pragma once
/** \brief is the master of all commands.
 * It knows executes the inialize, execute, and end functions
 * Right now, it only supports parallel commands
 * Sequential commands could be added by copying the logic from CommandGroups
 */

#include "LinkedList.h"
#include "Command.h"

class Scheduler {
  public:
    /** \brief pass the master command to the constructor
     * \param[_in] the master command to control everything
     * */
    Scheduler(Command *command);

    /** \brief adds a command to the command group
     * \param[_in] command the pointer
     */
    void addCommand(Command *command);

    /** \brief runs all the commands.
     * Iterates through the list of commands and runs them.
     * If they are done, it removes them from the list of commands.
     */
    void run();

  private:

    /** \brief singleton instance */
    static Scheduler *instance;

    /** \brief list of commands */
    LinkedList<Command *> commands;
};
