class Controller
{
public:
    double kp;
    // double ki;
    double kd;
    double e_d;

    Controller(double p, double d);
    double get_u(double r, double x, double u_up, bool back);
    ~Controller();
};