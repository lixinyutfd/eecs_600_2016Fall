// ur10_reachability.cpp
// XinyuLi, September 2016
// search for reachability for flange over x_range = [-1.5,1.5] , y_range= [-1.5,1.5] at z_range = ...


#include <ur_fk_ik/ur_kin.h> 
#include <fstream>
#include <string>
using namespace std;
double Ground = 0.0;
double Bin_top = 0.724275; 
double Linear_tray_top = 0.950316; 
double Conv_top = 0.903960; 
double AGV_top = 0.750201; 
double Arm_base = 1.099893;

int main(int argc, char **argv){
	ros::init(argc, argv, "ur10_reachability");
	double x_des,y_des,z_des;
    double x_min = -1.5;
    double x_max = 1.5;
    double y_min = -1.5;
    double y_max = 1.5;
    Eigen::Vector3d n_des,t_des,b_des;

    b_des<<0,0,-1; //tool flange pointing down
    n_des<<0,0,1; //x-axis pointing forward...arbitrary
    t_des = b_des.cross(n_des); //consistent right-hand frame

    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;

    std::vector<double> z;
    z.push_back(Arm_base - Ground);
    z.push_back(Arm_base - Bin_top);
    z.push_back(Arm_base - Linear_tray_top);
    z.push_back(Arm_base - Conv_top);
    z.push_back(Arm_base - AGV_top);

    //double z_low = 0.0;
    //double z_high = 0.1;
    double dx = 0.05;
    double dy = 0.05;
    Eigen::Vector3d p_des;
    Eigen::Affine3d A_fwd_DH;
    A_fwd_DH.linear() = R_des;
    std::vector<Eigen::VectorXd> q6dof_solns;
    int nsolns;
    std::vector<Eigen::Vector3d> reachable;
    UR10IkSolver ik_solver;
    std::string namelist[] = {"GROUND", "BIN_TOP", "TRAY_TOP", "CONV_TOP", "AGV_TOP"};

	for (int i = 0; i<z.size();i++){
		reachable.clear();
		for (double x_des = x_min;x_des<x_max;x_des+=dx) {
            for (double y_des = y_min; y_des<y_max; y_des+=dy) {
            	
	            p_des[0] = x_des;
	            p_des[1] = y_des;
	            p_des[2] = z[i];
	            A_fwd_DH.translation() = p_des;
	            nsolns = ik_solver.ik_solve(A_fwd_DH,q6dof_solns);
	            if (nsolns>0) { //test grasp pose:
                    ROS_INFO("soln at x,y = %f, %f",p_des[0],p_des[1]);
                    reachable.push_back(p_des);
                }
	       	}
        }
        ROS_INFO("saving the results...");
    	int temp_n = reachable.size();
    	ofstream outfile;

    	const char * name = namelist[i].c_str();
    	outfile.open(name);
    	//ROS_INFO("temp_n = %d", temp_n);
    	for (int j=0;j<temp_n;j++) {
        	p_des = reachable[j];
        	outfile<<p_des[0]<<", "<<p_des[1]<<endl;
    	}
    	outfile.close(); 
    	ROS_INFO("there?");
    }

    /*ROS_INFO("saving the results...");
    nsolns = reachable.size();
    ofstream outfile;
    outfile.open("ur10_reachable_x_y");
    for (int i=0;i<nsolns;i++) {
        p_des = reachable[i];
        outfile<<p_des[0]<<", "<<p_des[1]<<endl;
    }
    outfile.close();*/
    ROS_INFO("finished! Lalalala....:)");
   	return 0;



}