//#include <eigen3/Eigen/Dense>
#include <iostream>

#include "Tests/test_my_math.h"
#include "robo.h"
using namespace RoboMath;
using namespace std;


void JustFoo()
{
    {
    //    checkTime chktmr2;
    //    double kekeke = 123;
    //    Matrix u_buff(4,4);
    //    scalar temp_s;
    //    for (counter i = 0; i < 6; i++)
    //    {
    //        for (counter k = 0; k < 6; k++)
    //        {
    //            for (counter m = 0; m < 6; m++)
    //            {
    //                for (counter j = i; j < 6; j++)
    //                {
    //                    temp_s = 0;
    //                    u_buff.MakeIdentity();
    //                    for (counter ctr = 0; ctr < j; ctr++)
    //                    {
    //                        if (ctr == (k - 1) && ctr == (m - 1))
    ////                            rrrr = u_buff * lo * me * op;
    //                            m5 = m1 * m2 * m3 * m4;
    //                        else if (ctr == (k - 1) || ctr == (m - 1))
    ////                            u_buff = u_buff * lo * op;
    //                            m5 = m1 * m2 * m3;
    //                        else
    ////                            u_buff = u_buff * lo;
    //                            m5 = m1 * m2;
    //                    }
    ////                    cout << "u buff ikmj " << i << k << m  << j << endl;
    ////                    u_buff.MatrixCout("u_buff");
    //                }
    //            }
    //        }
    //    }
    }


//    Eigen::Matrix4Xd m1(4,4);
//    Eigen::Matrix4Xd m2(4,4);
//    Eigen::Matrix4Xd m3(4,4);
//    Eigen::Matrix4Xd m4(4,4);
//    Eigen::Matrix4Xd m5(4,4);
//    m1 <<  1,1,1,1,
//           1,1,1,1,
//           1,1,1,1,
//           1,1,1,1;
//    m2 <<  2,2,2,2,
//           2,2,2,2,
//           2,2,2,2,
//           2,2,2,2;
//    m3 <<  3,3,3,3,
//           3,3,3,3,
//           3,3,3,3,
//           3,3,3,3;
//    m4 <<  4,4,4,4,
//           4,4,4,4,
//           4,4,4,4,
//           4,4,4,4;
//    checkTime chktmr1;
//    for (int i = 0; i < 100; i++)
//        for (int j = 0; j < 100; j++)
//                m5 = m1 * m2 * m3;
//    cout << "Duration 1000000 matrix multiplication Eigen = " << chktmr1.duration().count() << " sek" << endl;

    Matrix lo{{1,1,1,1},
              {1,1,1,1},
              {1,1,1,1},
              {1,1,1,1}};
    Matrix me{{2,2,2,2},
              {2,2,2,2},
              {2,2,2,2},
              {2,2,2,2}};
    Matrix op{{3,3,3,3},
              {3,3,3,3},
              {3,3,3,3},
              {3,3,3,3}};
    Matrix rrrr(4,4);
    CheckTime chktmr;
    for (int i = 0; i < 100; i++)
    {
        for(int j = 0; j < 100; j++)
        {
            rrrr =  lo * me * op;
        }
    }
    cout << "Duration 1000000 matrix multiplication MY = " << chktmr.duration().count() << " sek" << endl;
    scalar sc1 = 0.26;
    Matrix4x4 mm1({sc1,sc1,sc1,sc1,
                   sc1,sc1,sc1,sc1,
                   sc1,sc1,sc1,sc1,
                   sc1,sc1,sc1,sc1});
    Matrix4x4 mm2({0.2,0.2,0.2,0.2,
                   0.2,0.2,0.2,0.2,
                   0.2,0.2,0.2,0.2,
                   0.2,0.2,0.2,0.2});
    Matrix4x4 mm3({0.3,0.3,0.3,0.3,
                   0.3,0.3,0.3,0.3,
                   0.3,0.3,0.3,0.3,
                   0.3,0.3,0.3,0.3});
    Matrix4x4 mmr({1,1,1,1,
                   1,1,1,1,
                   1,1,1,1,
                   1,1,1,1});
    CheckTime chktmr3;
    int ctr11 = 0;
    for (int i = 0; i < 100; i++)
    {
        for(int j = 0; j < 100; j++)
        {
            ctr11++;
            mmr = mmr * mm1 * mm2 * mm3;
        }
    }
    cout << "Duration 1000000 matrix multiplication 4x4 = " << chktmr3.duration().count() << " sek" << endl;
    mmr.MatrixCout("mmr");
    cout << "ctr = " << ctr11 << endl;
}
