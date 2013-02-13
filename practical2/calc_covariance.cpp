#include<iostream>
#include<cmath>

using namespace std;

int main(int argc, char *argv[])
{
    int n = 10;

    /*int x[n], y[n];
    for(int i = 0; i < n; i++)
    {
         x[i] = atoi(argv[i+1].c_str());
         y[i] = atoi(argv[i+1+n].c_str());
    }*/
/*
Pos = [(3.1, 0.4),
              (2.8, -0.7),
             (4.1, 0),
             (2, 1.4),
             (3.4, 1.2),
             (4.1, 0),
             (2.4, 0.6),
             (4.5, 1.2),
             (2.6, 0.5),
             (1.8, 0.6)
*/
    float x[] = {3.1, 2.8, 4.1, 2, 3.4, 4.1, 2.4, 4.5, 2.6, 1.8};
    float y[] = {0.4, -0.7, 0, 1.4, 1.2, 0, 0.6, 1.2, 0.5, 0.6};
    float u[] = {36.0, 9.0, 11.0, 10.0, 0.0, -14.0, -20.0, -11.0, -16.0, 0.0};

    float sx = 0;
    float sy = 0;
    float su = 0;

    for(int i = 0; i < n; i++)
    {
    	sx += x[i];
    	sy += y[i];
    	su += u[i];
    }
    float xm = sx / n;
    float ym = sy / n;
    float um = su / n;

    float p[5];

    float s1 = 0, s2 = 0, s3 = 0, s4 = 0;
    for(int i = 0; i < n; i++)
    {
    	s1 += (x[i] - xm) * (x[i] - xm);
    	s2 += (x[i] - xm) * (y[i] - ym);
    	s3 += (y[i] - ym) * (y[i] - ym);
    	s4 += (u[i] - um) * (u[i] - um);
    }

    p[0] = s1 / n;
    p[1] = s2 / n;
    p[2] = s2 / n;
    p[3] = s3 / n;
    p[4] = s4 / n;
    
    for(int i = 0; i < 4; i++)
    {
        cout << "p[" << i << "] = " << p[i] << endl;
        
    }
    cout << endl;
    cout << "std x: " << sqrt(p[0]) << endl;
    cout << "std y: " << sqrt(p[3]) << endl;
    cout << "std angle: " << sqrt(p[4]) << endl;

    
    scanf("%d",p[0]);
    return 0;
    
}
