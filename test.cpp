#include <iostream>
#include <string>

int main() {

	std::string str = R"(holes_static=[(2:2,5:4,5:2);(2:11,2:14,7:14,7:11);(15:3,15:12,16:12,18:3)]
holes_dynamic=[(5:7,5:8,6:8,6:7);(10:10,10:11,11:11,11:10);(10:3,10:4,11:4,11:3)]
path=
0.0:1:1:0.0:CHECKPOINT_REACHED
0.2:5:1:0.0:CHECKPOINT_REACHED
1.5:7:3:0.7854:CHECKPOINT_REACHED
1.7:7:6:1.5708:CHECKPOINT_REACHED
2.8:6:7:2.3562:CHECKPOINT_NOT_AVAILABLE
3.3:9:7:0.0:MOVE
3.6:14:7:0.0:SEARCH_TARGET
3.9:14:9:1.5708:BYPASSING_OBSTACLE
5.5:14:11:1.5708:NOT_SET)";

	std::cout << str << std::endl;
}
