#include "End.h"
#include "RealMouse.h"

End::End() : Command("end") {}

void End::end(){
  Serial1.println("DONE.");
}
