#include "perls-lcmtypes++/perllcm/matrix_d_t.hpp"

#include "origin_state.h"

// indices are: k_i -- newest state
//              p_i -- latest delayed state
//              d_i -- all other past states
//              o_i -- origin state (could be in p_i or d_i)


void perls::Osm_accumulator::add_block_state (const Eigen::MatrixXd& Lam, const Eigen::VectorXd& et, 
        const std::string& new_key)
{
    if (_eta.size () < _Ns + _index.size ()) { // warning comp unsigned/signed
        std::cerr << "{add_origin_state_packet} Lambda/eta are too small!" << std::endl;
        return;
    }

    if (_index.size () == _Ns) { // overwrite entire block
        _Lambda.topLeftCorner (_Nb,_Nb) = Lam.topLeftCorner (_Nb,_Nb);
        _eta.head (_Nb) = et.head (_Nb);
    }
    else { // shift delayed elements, add new block
        int Nd = _index.size () - _Ns;

        Eigen::MatrixXd Ldd = _Lambda.block (_Ns,_Ns,Nd,Nd);
        _Lambda.block (_Nb,_Nb,Nd,Nd) = Ldd;                               // d_i,d_i
        _Lambda.block (_Ns,_Nb,_Ns,Nd) = _Lambda.block (0,_Ns,_Ns,Nd);     // p_i,d_i
        _Lambda.block (_Nb,_Ns,Nd,_Ns) = _Lambda.block (_Ns,0,Nd,_Ns);     // d_i,p_i
        _Lambda.block (_Ns,_Ns,_Ns,_Ns) = Lam.bottomRightCorner (_Ns,_Ns); // p_i,p_i
        _Lambda.block (0,_Ns,_Ns,_Ns) = Lam.topRightCorner (_Ns,_Ns);      // k_i,p_i
        _Lambda.block (_Ns,0,_Ns,_Ns) = Lam.bottomLeftCorner (_Ns,_Ns);    // p_i,k_i
        _Lambda.topLeftCorner (_Ns,_Ns) = Lam.topLeftCorner (_Ns,_Ns);     // k_i,k_i

        Eigen::VectorXd ed = _eta.segment (_Ns,Nd);
        _eta.segment (_Nb,Nd) = ed;             // d_i
        _eta.segment (_Ns,_Ns) = et.tail (_Ns); // p_i
        _eta.head (_Ns) = et.head (_Ns);        // k_i
    }
    _index.prepend (new_key, _Ns);

    //write_accumulator ();
}

void perls::Osm_accumulator::add_single_state (const Eigen::MatrixXd& Lam, const Eigen::VectorXd& et, 
        const std::string& new_key)
{
    if (_eta.size () < _Ns + _index.size ()) { // warning comp unsigned/signed
        std::cerr << "{add_origin_state_packet} Lambda/eta are too small!" << std::endl;
        return;
    }

    // this adds the single state in *independent* of all other delayed states
    if (_index.empty ()) { // add single state
        _Lambda.topLeftCorner (_Ns,_Ns) = Lam.topLeftCorner (_Ns,_Ns);
        _eta.head (_Ns) = et.head (_Ns);

        _index.add (new_key, _Ns);
    }
    else { // shift delayed elements, add new block 
        int Nd = _index.size ();

        Eigen::MatrixXd Ldd = _Lambda.topLeftCorner (Nd,Nd);
        _Lambda.block (_Ns,_Ns,Nd,Nd) = Ldd;                           // d_i,d_i
        _Lambda.block (0,_Ns,_Ns,Nd) = Eigen::MatrixXd::Zero (_Ns,Nd); // k_i,d_i
        _Lambda.block (_Ns,0,Nd,_Ns) = Eigen::MatrixXd::Zero (Nd,_Ns); // d_i,k_i
        _Lambda.topLeftCorner (_Ns,_Ns) = Lam.topLeftCorner (_Ns,_Ns); // k_i,k_i

        Eigen::VectorXd ed = _eta.segment (_Ns,Nd);
        _eta.segment (_Ns,Nd) = ed;      // d_i
        _eta.head (_Ns) = et.head (_Ns); // k_i

        _index.prepend (new_key, _Ns);
    }
}

void perls::Osm_accumulator::add_osp (const Eigen::MatrixXd& osp_Lam, const Eigen::VectorXd& osp_eta,
        int new_no, int org_no)
{
    std::cout << "client index " << _index << std::endl;
    std::string new_key = boost::lexical_cast<std::string>(new_no);
    std::string org_key = boost::lexical_cast<std::string>(org_no);

    // TODO: CHECK TO BE SURE WE DONT ALREADY HAVE NEW_KEY SOMEHOW
    std::cout << "{Osm_accumulator} received tol ind = " << new_key << ", org ind = " << org_key << std::endl;

    int sz = _index.size ();
    if (sz == 0) { // add both origin and new key
        std::cout << "adding single state first" << std::endl;
        add_single_state (osp_Lam.bottomRightCorner (_Ns,_Ns), osp_eta.tail (_Ns), org_key);
        std::cout << "adding block state" << std::endl;
        add_block_state (osp_Lam, osp_eta, new_key);
        return;
    }

    // segment newly received message --- joint marginal [k o]
    Eigen::MatrixXd Lkk_r = osp_Lam.topLeftCorner (_Ns,_Ns) ;
    Eigen::MatrixXd Lko_r = osp_Lam.topRightCorner (_Ns,_Ns);
    Eigen::MatrixXd Loo_r = osp_Lam.bottomRightCorner (_Ns,_Ns);
    Eigen::VectorXd ek_r  = osp_eta.head (_Ns);
    Eigen::VectorXd eo_r  = osp_eta.tail (_Ns);

    std::vector<int> o_i = _index[org_key];
    if (o_i.empty ()) {
        std::cerr << "{Osm_client_accumulator} have not recieved origin!" << std::endl;
        exit (EXIT_FAILURE);
    }
    if (o_i.front () == 0) { // origin is last received state, o == p
        std::cout << "{Osm_client_accumulator} origin is the previous state" << std::endl;

        // segment last distribution --- [p d]
        int Nd = sz - _Ns;
        Eigen::MatrixXd Lpd = _Lambda.block (0,_Ns,_Ns,Nd);
        Eigen::MatrixXd Ldp = Lpd.transpose ();
        Eigen::MatrixXd Ldd = _Lambda.block (_Ns,_Ns,Nd,Nd);
        Eigen::MatrixXd Ldd_inv = ldlt_inverse (Ldd);
        Eigen::VectorXd ed = _eta.segment (_Ns,Nd);

        // update the joint --- [k p] 
        Eigen::MatrixXd Lam (_Nb,_Nb);                    
        Lam.topLeftCorner (_Ns,_Ns) = Lkk_r;                       // k_i,k_i
        Lam.topRightCorner (_Ns,_Ns) = Lko_r;                      // k_i,p_i
        Lam.bottomLeftCorner (_Ns,_Ns) = Lko_r.transpose ();       // p_i,k_i
        Lam.bottomRightCorner (_Ns,_Ns) = Loo_r + Lpd*Ldd_inv*Ldp; // p_i,p_i

        Eigen::VectorXd et (_Nb);
        et.head (_Ns) = ek_r;                  // k_i
        et.tail (_Ns) = eo_r + Lpd*Ldd_inv*ed; // p_i

        add_block_state (Lam, et, new_key);
    }
    else { // origin is not the last received state, o < p
        std::cout << "{Osm_client_accumulator} origin is NOT the previous state" << std::endl;

        // joint marginal of [p o] from existing distribution
        Eigen::MatrixXd Lam_m;
        Eigen::VectorXd eta_m;

        std::vector<int> k_i = _index.child(0)[""];
        std::vector<int> m_i (k_i.size ()+o_i.size ());
        std::merge (k_i.begin (),k_i.end (),o_i.begin (),o_i.end (),m_i.begin ());

        perls::marg_info_exclude (Lam_m,eta_m,m_i,_Lambda.topLeftCorner (sz,sz),_eta.head (sz)); 
        
        Eigen::MatrixXd Lpp_m = Lam_m.topLeftCorner (_Ns,_Ns);
        Eigen::MatrixXd Lpo_m = Lam_m.topRightCorner (_Ns,_Ns);
        Eigen::MatrixXd Lop_m = Lpo_m.transpose ();
        Eigen::MatrixXd Loo_m = Lam_m.bottomRightCorner (_Ns,_Ns);
        Eigen::VectorXd ep_m = eta_m.head (_Ns);
        Eigen::VectorXd eo_m = eta_m.tail (_Ns);

        // solve intermediate --- marginal of [k p' o] with osp
        Eigen::MatrixXd Lpp_p_inv = Lop_m.inverse ()*(Loo_m - Loo_r)*Lpo_m.inverse ();
        Eigen::MatrixXd Lpp_p  = Lpp_p_inv.inverse ();
        Eigen::MatrixXd Lkp = -Lko_r*(Lpp_p_inv*Lpo_m).inverse ();
        Eigen::MatrixXd Lpk = Lkp.transpose ();
        Eigen::MatrixXd Lkk = Lkk_r + Lkp*Lpp_p_inv*Lpk;
        Eigen::VectorXd ep_p = (Lop_m*Lpp_p_inv).inverse ()*(eo_m - eo_r);
        Eigen::VectorXd ek = ek_r + Lkp*Lpp_p_inv*ep_p;

        // do we have additional delayed states between p and o?
        //    --- in this case we have [k p i o] (i for intermediate)
        Eigen::MatrixXd Lpp;
        Eigen::VectorXd ep;
        int Ni = (o_i.front () - 1) - k_i.back (); // should never be less than 0
        if (Ni > 0) {
            std::cout << "{Osm_client_accumulator} delayed states exist between p and o" << std::endl;

            Eigen::MatrixXd Lpi = _Lambda.block (0,_Ns,_Ns,Ni);
            Eigen::MatrixXd Lip = Lpi.transpose ();
            Eigen::MatrixXd Lii = _Lambda.block (_Ns,_Ns,Ni,Ni);
            Eigen::VectorXd ei = _eta.segment (_Ns,Ni);

            Eigen::MatrixXd Lii_inv = ldlt_inverse (Lii);
            Lpp = Lpp_p + Lpi*Lii_inv*Lip;
            ep = ep_p + Lpi*Lii_inv*ei;
        }
        else {
            std::cout << "{Osm_client_accumulator} delayed states do not exist between p and o" << std::endl;
            Lpp = Lpp_p;
            ep = ep_p;
        }

        // reassemble Lambda and eta with new elements --- [k p d] 
        Eigen::MatrixXd Lam (_Nb,_Nb);
        Lam.topLeftCorner (_Ns,_Ns) = Lkk;     // k_i,k_i
        Lam.topRightCorner (_Ns,_Ns) = Lpk;    // p_i,k_i
        Lam.bottomLeftCorner (_Ns,_Ns) = Lkp;  // k_i,p_i
        Lam.bottomRightCorner (_Ns,_Ns) = Lpp; // p_i,p_i

        Eigen::VectorXd et (_Nb);
        et.head (_Ns) = ek; // k_i
        et.tail (_Ns) = ep; // p_i

        add_block_state (Lam, et, new_key);
    }
}

// idea is to marginalize out states between new_no and org_no, then
// return the block corresponding to new_no and org_no
void perls::Osm_accumulator::block_state (Eigen::MatrixXd& Lambda, Eigen::VectorXd& eta, 
        int new_no, int org_no) 
{
    std::cout << "{Origin_state} extracting block for " << new_no 
       << ", " << org_no << std::endl;
    std::string new_key = boost::lexical_cast<std::string>(new_no);
    std::string org_key = boost::lexical_cast<std::string>(org_no);

    std::vector<int> n_i = _index[new_key];
    std::vector<int> o_i = _index[org_key];

    int Ni = (o_i.front ()-1) - n_i.back ();
    std::vector<int> b_i (Ni);
    for (int i=0;i<Ni;++i) b_i[i] = (n_i.back ()+1) + i;
    std::vector<int> f_i (n_i.front ());
    for (int i=0;i<n_i.front ();++i) f_i[i] = i;

    std::vector<int> m_i (f_i.size () + b_i.size ());
    std::merge (f_i.begin (),f_i.end (),b_i.begin (),b_i.end (),m_i.begin ());

    Eigen::MatrixXd Lam_m;
    Eigen::VectorXd eta_m;
    int sz = _index.size ();
    perls::marg_info_over (Lam_m,eta_m,m_i,_Lambda.topLeftCorner (sz,sz),_eta.head (sz)); 

    Lambda = Lam_m.topLeftCorner (_Nb, _Nb);
    eta = eta_m.head (_Nb);

    /*
    std::cout << "BLOCK " << std::endl << Lam_m.topLeftCorner (2*_Nb,2*_Nb) << std::endl;
    std::cout << "BLOCK " << std::endl << Lambda << std::endl;

    Eigen::MatrixXd Sigma = ldlt_inverse (_Lambda.topLeftCorner (sz,sz));
    Eigen::VectorXd mu = Sigma * _eta.head (sz);
    std::cout << "BLOCK mu(org_no) = " << mu (o_i) << std::endl;
    std::cout << "BLOCK mu(new_no) = " << mu (n_i) << std::endl;

    std::cout << "BLOCK o_i: " << o_i << std::endl;
    std::cout << o_i[0] << " AND " << o_i[0]+_Ns << " AND " << o_i[0]-_Ns << std::endl;
    std::cout << "BLOCK mu.segment (o_i[0],_Ns) " << std::endl << mu.segment (o_i[0],_Ns) << std::endl;;
    */
}

void perls::Osm_accumulator::write_accumulator ()
{
    // write information matrix to log
    int sz = _index.size ();
    perllcm::matrix_d_t Lam_lcm;
    Lam_lcm.nrows = sz;
    Lam_lcm.ncols = sz;
    Lam_lcm.n = sz*sz;
    Lam_lcm.data.resize (sz*sz);
    Eigen::MatrixXd Lam_mat = _Lambda.topLeftCorner(sz,sz);
    std::copy (Lam_mat.data (), Lam_mat.data () + sz*sz, Lam_lcm.data.begin ());

    int Lam_maxlen = Lam_lcm.getEncodedSize ();
    double *Lam_data = new double[Lam_maxlen];
    Lam_lcm.encode (Lam_data, 0, Lam_maxlen);
    lcm::LogEvent Lam_event;
    Lam_event.timestamp = sz;
    Lam_event.channel = "Lambda";
    Lam_event.datalen = Lam_maxlen;
    Lam_event.data = Lam_data;
    _lcm_event_log.writeEvent (&Lam_event);
    delete[] Lam_data;

    // write information vector to log
    perllcm::matrix_d_t eta_lcm;
    eta_lcm.nrows = sz;
    eta_lcm.ncols = 1;
    eta_lcm.n = sz;
    eta_lcm.data.resize (sz);
    Eigen::VectorXd eta_mat = _eta.head (sz);
    std::copy (eta_mat.data (), eta_mat.data () + sz, eta_lcm.data.begin ());

    int eta_maxlen = eta_lcm.getEncodedSize ();
    double *eta_data = new double[eta_maxlen];
    eta_lcm.encode (eta_data, 0, eta_maxlen);
    lcm::LogEvent eta_event;
    eta_event.timestamp = sz;
    eta_event.channel = "eta";
    eta_event.datalen = eta_maxlen;
    eta_event.data = eta_data;
    _lcm_event_log.writeEvent (&eta_event);
    delete[] eta_data;

    // write indices to log
    perllcm::matrix_d_t ind_lcm;
    ind_lcm.nrows = _index.nchildren ();
    ind_lcm.ncols = 1;
    ind_lcm.n = _index.nchildren ();
    ind_lcm.data.resize (_index.nchildren ());
    for (unsigned int i=0;i<_index.nchildren ();++i) 
        ind_lcm.data[i] = boost::lexical_cast<int>(_index.child (i).label ());

    int ind_maxlen = ind_lcm.getEncodedSize ();
    int *ind_data = new int[ind_maxlen];
    ind_lcm.encode (ind_data, 0, ind_maxlen);
    lcm::LogEvent ind_event;
    ind_event.timestamp = sz;
    ind_event.channel = "index";
    ind_event.datalen = ind_maxlen;
    ind_event.data = ind_data;
    _lcm_event_log.writeEvent (&ind_event);
    delete[] ind_data;
}
