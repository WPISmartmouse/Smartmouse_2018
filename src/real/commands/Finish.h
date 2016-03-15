#pragma once
#ifdef EMBED

#include "CommanDuino.h"
#include "AbstractMaze.h"

class Finish : public Command {
  public:
    Finish(AbstractMaze *maze);
    void initialize();
    bool isFinished();

  private:
    AbstractMaze *maze;

};
#endif
