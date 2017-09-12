#pragma once

#include "TimerInterface.h"

/**
 * \brief this class is the very core of the framework
 * commands are initialized once, then run until they're done
 * completed commands are removed the scheduler
 */
class Command {

public:

  static void setTimerImplementation(TimerInterface *timer);
  static TimerInterface *getTimerImplementation();

  Command();

  Command(const char *name);

  virtual ~Command();

  void setTimeout(unsigned long timeout);

  unsigned long getTime();

  bool isTimedOut();

  /** \brief run once in the first iteration of the command's life.
   * this will be overrided by individual commands
   * */
  virtual void initialize();

  virtual void _initialize();

  /** \brief stuff to do over and over each iteration.
   * this will be overrided by individual commands
   */
  virtual void execute();

  virtual void _execute();

  /** \brief checked every iteration to see if we're done here.
   * this will be overrided by individual commands
   * @return is the function finished
   */
  virtual bool isFinished() = 0;

  /** \brief called once at the end, once isFinished() returned true.
   * this will be overrided by individual commands.
   */
  virtual void end();

  virtual void _end();

  /** \brief actually does the  excuting.
   * @return if command is finished
   */
  bool cycle();

  /** \brief check if the command is running.
   *  @return if command is still running
   */
  bool isRunning();

  /** \brief comparator overload for !=. used when checking if command is in list */
  bool operator!=(const Command &other);

  /** \brief used by command group to organize commands */
  bool inParallel;

  /** \brief for convenient printing */
  const char *name;

  static TimerInterface *timer;

  bool initialized, running;
  unsigned long timeout;
  unsigned long startTime;
};
