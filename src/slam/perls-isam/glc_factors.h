/**
 * @file slam_factors.h
 * @brief User defined factors
 *
 */
#include <isam/isam.h>

#include "perls-math/ssc.h"

// NOTE: When you add a new factor, modify gparser.cpp/h to enable proper graph loading
// Unfortunately, currently no loading from native isam is supported.

using namespace Eigen;
using namespace std;


namespace isam {
   
    // generic linear constraint factor
    class GLC_Factor : public Factor {
        const vector<Pose3d_Node*> _poses;

      public:
        const VectorXd _x;
        const MatrixXd _U;
        
        /**
         * Constructor.
         * @param 
         */
      GLC_Factor(vector<Pose3d_Node*> poses, const VectorXd& x, const MatrixXd& U)
          : Factor("GLC_Factor", U.rows(), Information(MatrixXd::Identity(U.rows(), U.rows()))),
            _poses(poses), _x(x), _U(U)
        {
            int np = _poses.size();
            _nodes.resize(np);
            for (int i=0; i<np; i++) {
                _nodes[i] = poses[i];
            }         
        }

        void initialize() {
           
        }
        
        // make faster by supplying jacobian
        Jacobian jacobian () {

            MatrixXd H = sqrtinf() * _U;
            VectorXd r = error(LINPOINT);
            Jacobian jac(r);
            int position = 0;
            int n_measure = dim();
            for (size_t i=0; i<_nodes.size(); i++) {
              int n_var = _nodes[i]->dim();
              MatrixXd Hi = H.block(0, position, n_measure, n_var);
              position += n_var;
              jac.add_term(_nodes[i], Hi);
            }
            
            return jac;    
        
        }

        VectorXd basic_error(Selector s = LINPOINT) const {

            int np = _poses.size();
            int n = np*6;

            VectorXd x_p(n);
            // fill x with the poses
            for (int i=0; i<np; i++) {
                x_p.segment<6>(i*6) = _poses[i]->vector(s);
            }
                    
            VectorXd err_wf = x_p - _x;
         
            // deal with difference in angles
            for(int i=0; i<err_wf.size(); i++) {
                if(i % 6 == 3 || i % 6 == 4 || i % 6 == 5)
                    err_wf(i) = standardRad(err_wf(i));
            } 
            VectorXd err = _U*err_wf;
            return err;
        }

        void write(ostream &out) const {
            Factor::write(out);
            out << " (";
            for (int i=0; i<_x.size(); i++)
                out << _x(i) << " ";
            out << ") {";
            for (int i=0; i<_U.rows(); i++)
                for (int j=0; j<_U.cols(); j++)
                    out << _U(i,j) << " ";
            out << "}";
        }

    };

    
    // root-shifted generic linear constraint factor
    class GLC_RS_Factor : public Factor {
        const vector<Pose3d_Node*> _poses;

      public:
        const VectorXd _x_rs;
        const MatrixXd _U;
        bool _inc_x_o_w;
        
        /**
         * Constructor.
         * @param 
         */
      GLC_RS_Factor(vector<Pose3d_Node*> poses, const VectorXd& x_rs, const MatrixXd& U, bool inc_x_o_w = false)
          : Factor("GLC_RS_Factor", U.rows(), Information(MatrixXd::Identity(U.rows(), U.rows()))),
            _poses(poses), _x_rs(x_rs), _U(U), _inc_x_o_w(inc_x_o_w)
        {
            int np = _poses.size();
            _nodes.resize(np);
            for (int i=0; i<np; i++) {
                _nodes[i] = poses[i];
            }
        }

        void initialize() {
            
        }
        
        
        // make faster by supplying jacobian
        Jacobian jacobian () {
        
            VectorXd x_rs;
            MatrixXd F;
            isamu_root_shift (x_rs, &F, _nodes, LINPOINT, _inc_x_o_w);        
            
            MatrixXd H = sqrtinf() * _U * F;
            VectorXd r = error(LINPOINT);
            
            Jacobian jac(r);
            int position = 0;
            int n_measure = dim();
            for (size_t i=0; i<_nodes.size(); i++) {
              int n_var = _nodes[i]->dim();
              MatrixXd Hi = H.block(0, position, n_measure, n_var);
              position += n_var;
              jac.add_term(_nodes[i], Hi);
            }
            
            return jac;    
        
        }
        

        VectorXd basic_error(Selector s = LINPOINT) const {

            VectorXd x_rs_p;
            isamu_root_shift (x_rs_p, NULL, _nodes, s, _inc_x_o_w);           
                  
            VectorXd err_rs = x_rs_p - _x_rs;
         
            // deal with difference in angles
            for(int i=0; i<err_rs.size(); i++) {
                if(i % 6 == 3 || i % 6 == 4 || i % 6 == 5)
                    err_rs(i) = standardRad(err_rs(i));
            } 
            VectorXd err = _U*err_rs;

            return err;
        }
        

        void write(ostream &out) const {
            Factor::write(out);
            out << " (";
            for (int i=0; i<_x_rs.size(); i++)
                out << _x_rs(i) << " ";
            out << ") {";
            for (int i=0; i<_U.rows(); i++)
                for (int j=0; j<_U.cols(); j++)
                    out << _U(i,j) << " ";
            out << "}";
        }

    };
    
    
    
    // root-shifted generic constraint factor
    class RS_Factor : public Factor {
        const vector<Pose3d_Node*> _poses;

      public:
        const VectorXd _z;
        bool _inc_x_o_w;
        
        /**
         * Constructor.
         * @param 
         */
      RS_Factor (vector<Pose3d_Node*> poses, const VectorXd& z, const Noise& noise, bool inc_x_o_w)
          : Factor("RS_Factor", z.size(), noise), _poses(poses), _z(z), _inc_x_o_w(inc_x_o_w) 
        {
            int np = _poses.size();
            _nodes.resize(np);
            for (int i=0; i<np; i++) {
                _nodes[i] = poses[i];
            }         
        }

        void initialize() {
           
        }
        
        
        // make faster by supplying jacobian
        Jacobian jacobian () {
        
            VectorXd x_rs;
            MatrixXd F;
            isamu_root_shift (x_rs, &F, _nodes, LINPOINT, _inc_x_o_w);         
            
            MatrixXd H = sqrtinf() * F;
            VectorXd r = error(LINPOINT);
            
            Jacobian jac(r);
            int position = 0;
            int n_measure = dim();
            for (size_t i=0; i<_nodes.size(); i++) {
              int n_var = _nodes[i]->dim();
              MatrixXd Hi = H.block(0, position, n_measure, n_var);
              position += n_var;
              jac.add_term(_nodes[i], Hi);
            }            
            return jac;    
        
        }
        

        VectorXd basic_error(Selector s = LINPOINT) const {

            VectorXd zp;
            isamu_root_shift (zp, NULL, _nodes, s, _inc_x_o_w);
        
            VectorXd err(zp.size());

            // calculate error
            err = zp - _z;
            
            // deal with difference in angles
            for(int i=0; i<err.size(); i++) {
                if(i % 6 == 3 || i % 6 == 4 || i % 6 == 5)
                    err(i) = standardRad(err(i));
            }    
            
            
            return err;
        }
        

        void write(ostream &out) const {
            Factor::write(out);
            out << " (";
            for (int i=0; i<_z.size(); i++)
                out << _z(i) << " ";
            out << ") " << noise_to_string(_noise);
        }

    };


} // namespace isam
