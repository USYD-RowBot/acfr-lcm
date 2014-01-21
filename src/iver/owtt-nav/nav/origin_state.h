#ifndef __ORIGIN_STATE_H__
#define __ORIGIN_STATE_H__

#include <boost/lexical_cast.hpp>
#include <lcm/lcm-cpp.hpp>

#include "eigen_utils.h"
#include "index.h"
#include "utils.h"

#define OSM_MAX_SIZE 500

namespace perls
{
    // origin state method assumes that the state grows according to a Markov
    // process and is only updated with local measurements of the *current*
    // state --- the information matrix is therefore tri-diagonal, and only
    // blocks corresponding to the current and latest delayed state are
    // modified

    class Osm_accumulator 
    {
      public:
        Osm_accumulator (int state_size) 
            : _Ns (state_size), _Nb (2*state_size),
            _lcm_event_log ("lcmlog-accumulator","w")
        {
            _Lambda = Eigen::MatrixXd::Zero (OSM_MAX_SIZE,OSM_MAX_SIZE);
            _eta = Eigen::VectorXd::Zero (OSM_MAX_SIZE);
        }
        virtual ~Osm_accumulator () {}

        Eigen::VectorXd mean () const 
        {
            int sz = _index.size ();
            return ldlt_inverse (_Lambda.topLeftCorner (sz,sz))*_eta.head (sz);
        }
        Eigen::MatrixXd cov () const 
        {
            int sz = _index.size ();
            return ldlt_inverse (_Lambda.topLeftCorner (sz,sz));
        }
        Eigen::MatrixXd Lambda () const {return _Lambda.topLeftCorner (_index.size (),_index.size ());}
        Eigen::VectorXd eta () const {return _eta.head (_index.size ());}

        int size () const {return _index.size ();}
        bool empty () const {return _index.empty ();}

        int state_order_no (unsigned int n) 
        {
            if (n > _index.nchildren () -1) return 0;
            return boost::lexical_cast<int>(_index.child (n).label ());
        }
        int find_state_no (int n) {return _index[boost::lexical_cast<std::string>(n)].front ()/_Ns;}

        void add_single_state (const Eigen::MatrixXd& Lam, const Eigen::VectorXd& et, const std::string& new_key);
        void add_block_state (const Eigen::MatrixXd& Lam, const Eigen::VectorXd& et, const std::string& new_key);
        void add_osp (const Eigen::MatrixXd& osp_Lam, const Eigen::VectorXd& osp_eta,
                int new_no, int org_no);

        void block_state (Eigen::MatrixXd& Lambda, Eigen::VectorXd& eta, int new_no, int org_no); 

      private:
        unsigned int _Ns; // state size
        unsigned int _Nb; // block size = 2 x state size
        Eigen::MatrixXd _Lambda;
        Eigen::VectorXd _eta;

        Index _index;

        // where Lambda/eta is the block of the info matrix/vector
        // corresponding to the current state and latest delayed state lifted
        // directly from the info matrix/vector
        void update_info (const Eigen::MatrixXd& Lam, const Eigen::VectorXd& et, const std::string& new_key);

        lcm::LogFile _lcm_event_log;
        void write_accumulator ();
        
        Osm_accumulator ();
    };
}


#endif // __ORIGIN_STATE_H__
