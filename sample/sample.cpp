//主函数
#include "sample.h"
#include "sample.hpp"
#include "debug.h"

using namespace cv;
using namespace std;

#ifdef SAMPLE
bool Sample::run()
{
    printf("Hello,sample")
}
#endif  //控制是否编译sample函数

int main()
{
    if(threshold)
        printf("Threshold:True");
    return 0;
}