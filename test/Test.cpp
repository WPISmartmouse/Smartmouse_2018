#include "Test.h"
#include "AbstractMaze.h"
#define RED     "\x1b[31m"
#define GREEN   "\x1b[32m"
#define YELLOW  "\x1b[33m"
#define BLUE    "\x1b[34m"
#define MAGENTA "\x1b[35m"
#define CYAN    "\x1b[36m"
#define RESET   "\x1b[0m"

TestResult::TestResult() : success(false), status(-1), msg("uninitialized"){}
TestResult::TestResult(bool success, int status, const char *msg) :
     success(success), status(status), msg(msg){}

TestResult results[100];

TestResult testOutOfBounds(){
  AbstractMaze maze;
  Node *n;
  int status = maze.get_node(&n, -1, -1);

  if (status != Node::OUT_OF_BOUNDS){
    return TestResult(false,-1,"Node out of Bounds");
  }
  else {
    return TestResult(true,0,"testOutOfBounds Passed!");
  }
}

int main(){
  int index = 0;
  results[index++] = testOutOfBounds();


  for (int i=0;i<index;i++){
    TestResult r = results[i];
    printf(YELLOW "[%s]  " RESET,r.msg);
    if (r.success){
      printf(GREEN "PASSED (%d)\n" RESET, r.status);
    }
    else {
      printf(RED "FAILED (%d)\n" RESET, r.status);
    }
  }
}
