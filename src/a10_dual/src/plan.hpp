
const double PI = 3.141592653589793;


class cosCurve
{
private:
    double a_;
    double w_;
    double p_;

public:
    auto getCurve(int count) -> double;  
    cosCurve(double a, double w, double p)
    {
        a_ = a;
        w_ = w;
        p_ = p;
    }  
    ~cosCurve() {} 
};



///���ܣ�����0->1���������ߡ��ɸ�������ļ��ٶȺ��ٶ��ж�����Ϊ���λ���������
//   ##��������##
//  Tc:���������ܹ���Ҫ��ʱ�䣬������ļ��ٶȺ��ٶȼ���
//   v:�ٶȣ����û����룬���캯����ʼ��
//   a:���ٶȣ����û����룬���캯����ʼ��
//  ta:���ٶ������ʱ�䣬��������ٶȺͼ��ٶȼ���õ�
class TCurve
{
private:
    double Tc_;
    double v_;
    double a_;
    double ta_;

public:
    auto getTCurve(int count) -> double;
    auto getCurveParam() -> void;
    auto getTc() -> double { return Tc_; };
    TCurve(double a, double v) { a_ = a; v_ = v; }
    ~TCurve() {}
};


//T�����߸ģ�����ֵΪ�������ٶȣ�����ٶȣ�Ŀ��λ�ã�
//tc_ָ�˶���Ŀ�����ʱ�䣻
//tm_ָ������ٶ��˶�����ʱ�䣻
//v_Ϊ����˶��ٶȣ�
//a_Ϊ����˶����ٶȣ�
//ta_ָ����ʱ�䣻
//p_ָ�����˶�λ�ã�
//pa_ָ�Ӽ�������Ҫ�˶���λ�á�
class TCurve2
{
private:
    double tc_;
    double tm_;
    double v_;
    double a_;
    double ta_;
    double p_;
    double pa_;

public:
    auto getTCurve(int count) -> double;
    auto getCurveParam() -> void;
    auto getTc() -> double { return tc_; };
    TCurve2(double a, double v, double p) { a_ = a; v_ = v; p_ = p; }
    ~TCurve2() {}
};






class GravComp
{
private:

    auto getTempFMatrix(double force_data_[6], double temp_[18]) -> void;
    auto getTempRMatrix(double pose_matrix_[9], double temp_[18]) -> void;
    auto getInverseRm(double rotation_matrix_[9], double inverse_rot_[9]) -> void;

public:

    auto getTorqueVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double torque_vector_[9]) -> void;
    auto getForceVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double force_vector_[9]) -> void;
    auto getFMatrix(double force_data1_[6], double force_data2_[6], double force_data3_[6], double f_matrix_[54]) -> void;
    auto getRMatrix(double pose_matrix1_[9], double pose_matrix2_[9], double pose_matrix3_[9], double r_matrix_[54]) -> void;
    auto getPLMatrix(double f_r_matrix_[54], double torque_force_data_[9], double P_L[6]) -> void;
    auto getCompFT(double current_pose_[16], double L_[6], double P_[6], double comp_f_[6]) -> void;

    auto savePLVector(const double P1_[6], const double L1_[6], const double P2_[6], const double L2_[6]) -> void;
    auto loadPLVector(double P1_[6], double L1_[6], double P2_[6], double L2_[6]) -> void;

};
