/**
 * @file user_factors.h
 * @brief User defined factors
 *
 */
#include <isam/isam.h>

#include "user_nodes.h"

// NOTE: When you add a new factor, modify gparser.cpp/h to enable proper graph loading
// Unfortunately, currently no loading from native isam is supported.

#pragma once
namespace Eigen {
    typedef Matrix<double, 1, 1> Matrix1d;
}


namespace isam {

    class Sonar3d {
      public:
        static const int dim = 3;
        static const char* name() {
            return "Sonar2d";
        }
      private:
        Pose2d m_pose;
    };


// A 2d constraint between sonar frames
    class Sonar2d_Factor : public FactorT<Pose2d> {
      const Pose3d_Node* _pose1;
      const Pose3d_Node* _pose2;
      Pose3d _sonar_frame1;
      Pose3d _sonar_frame2;
    public:

      /**
       * Constructor.
       * @param pose1 The pose from which the measurement starts.
       * @param pose2 The pose to which the measurement extends.
       * @param measure The relative measurement from pose1 to pose2 (pose2 in pose1's frame).
       * @param noise The 3x3 square root information matrix (upper triangular).
       * @sonar_frame1 the transformation from pose1 to the sonar frame.
       * @sonar_frame2 the transfomration from pose2 to the sonar frame.
       */
      Sonar2d_Factor(Pose3d_Node* pose1, Pose3d_Node* pose2,
                     const Pose2d& measure, const Noise& noise,
                     Pose3d sonar_frame1, Pose3d sonar_frame2)
        : FactorT<Pose2d>("Sonar2d_Factor", 3, noise, measure), 
          _pose1(pose1), 
          _pose2(pose2),
          _sonar_frame1(sonar_frame1),
          _sonar_frame2(sonar_frame2) 
      {
        _nodes.resize(2);
        _nodes[0] = pose1;
        _nodes[1] = pose2;
      }

      void initialize() {
      }

      Eigen::VectorXd basic_error(Selector s = LINPOINT) const {

        // @todo  add predicted sonar transformation
        // @todo  add sonar transformation from vehicle body

        const Pose3d& p1 = _pose1->value(s);
        const Pose3d& p2 = _pose2->value(s);
        Pose3d predicted = (p2.oplus(_sonar_frame2)).ominus(p1.oplus(_sonar_frame1));

        Eigen::VectorXd p(3);
        p << predicted.x(), predicted.y(), predicted.yaw(); // Predicted
        Eigen::VectorXd err(3);
        err << p(0)-_measure.x(), p(1)-_measure.y(), p(2)-_measure.t();
        err(2) = standardRad(err(2));

        return err;
      }

      void write(std::ostream &out) const {
        Factor::write(out);
        out << " " << _measure << " " << noise_to_string(_noise) << " " << _sonar_frame1  << " " << _sonar_frame2;
      }

    };

// xyz factor
    class Pose3d_xyz_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Vector3d _prior;
        /**
         * Constructor.
         * @param pose The pose node the prior acts on.
         * @param prior The actual prior measurement.
         * @param sqrtinf The 3x3 square root information matrix (upper triangular).
         */
      Pose3d_xyz_Factor(Pose3d_Node* pose, const Eigen::Vector3d& prior, const Noise& noise)
          : Factor("Pose3d_xyz_Factor", 3, noise), _pose(pose), _prior(prior)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial prior is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(3);
            err << pose.x() - _prior(0), pose.y() - _prior(1), pose.z() - _prior(2);
            err(1) = standardRad(err(1));
            err(2) = standardRad(err(2));
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _prior(0) << ", "<< _prior(1) << ", " << _prior(2) << ") " << noise_to_string(_noise);
        }

    };

// h factor
    class Pose3d_h_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Matrix1d _prior;
        /**
         * Constructor.
         * @param pose The pose node the prior acts on.
         * @param prior The actual prior measurement.
         * @param sqrtinf The 1x1 square root information matrix (upper triangular).
         */
      Pose3d_h_Factor(Pose3d_Node* pose, const Eigen::Matrix1d& prior, const Noise& noise)
          : Factor("Pose3d_h_Factor", 1, noise), _pose(pose), _prior(prior)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial prior is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(1);
            err << pose.yaw() - _prior(0);
            err(0) = standardRad(err(0));
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _prior(0) << ") " << noise_to_string(_noise);
        }

    };

// z factor
    class Pose3d_z_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Matrix1d _prior;
        /**
         * Constructor.
         * @param pose The pose node the prior acts on.
         * @param prior The actual prior measurement.
         * @param sqrtinf The 1x1 square root information matrix (upper triangular).
         */
      Pose3d_z_Factor(Pose3d_Node* pose, const Eigen::Matrix1d& prior, const Noise& noise)
          : Factor("Pose3d_z_Factor", 1, noise), _pose(pose), _prior(prior)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial prior is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(1);
            err << pose.z() - _prior(0);
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _prior(0) << ") " << noise_to_string(_noise);
        }

    };




// rp factor
    class Pose3d_rp_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Vector2d _prior;
        /**
         * Constructor.
         * @param pose The pose node the prior acts on.
         * @param prior The actual prior measurement.
         * @param sqrtinf The 2x2 square root information matrix (upper triangular).
         */
      Pose3d_rp_Factor(Pose3d_Node* pose, const Eigen::Vector2d& prior, const Noise& noise)
          : Factor("Pose3d_rp_Factor", 2, noise), _pose(pose), _prior(prior)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial prior is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(2);
            err << pose.roll() - _prior(0), pose.pitch() - _prior(1);
            err(0) = standardRad(err(0));
            err(1) = standardRad(err(1));
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _prior(0) << ", "<< _prior(1) << ") " << noise_to_string(_noise);
        }

    };

// xy factor
    class Pose3d_xy_Factor : public Factor {
        const Pose3d_Node* _pose;

      public:
        const Eigen::Vector2d _prior;
        /**
         * Constructor.
         * @param pose The pose node the prior acts on.
         * @param prior The actual prior measurement.
         * @param sqrtinf The 2x2 square root information matrix (upper triangular).
         */
      Pose3d_xy_Factor(Pose3d_Node* pose, const Eigen::Vector2d& prior, const Noise& noise)
          : Factor("Pose3d_xy_Factor", 2, noise), _pose(pose), _prior(prior)
        {
            _nodes.resize(1);
            _nodes[0] = pose;
        }

        void initialize() {
            // Partial prior is not used for initialization
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            // associated pose x,y,z,h,p,r
            const Pose3d& pose = _pose->value(s);
            Eigen::VectorXd err(2);
            err << pose.x() - _prior(0), pose.y() - _prior(1);
            return err;
        }

        void write(std::ostream &out) const {
            Factor::write(out);
            out << " (" << _prior(0) << ", "<< _prior(1) << ") " << noise_to_string(_noise);
        }

    };

    class Plane3d_Factor : public FactorT<Plane3d> {
        Plane3d_Node* _surf;

      public:
        Plane3d_Factor (Plane3d_Node *surf, const Plane3d& measure, const Noise& noise, Pose3d *senxform1 = NULL, Pose3d *senxform2 = NULL)
            : FactorT<Plane3d> ("Plane3d_Factor", Plane3d::dim, noise, measure), _surf (surf) {
            _nodes.resize(1);
            _nodes[0] = surf;
        }

        void initialize() {
            if (!_surf->initialized()) {
                Plane3d predict = _measure;
                _surf->init(predict);
            }
        }

        Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
            Eigen::VectorXd err = _nodes[0]->vector(s) - _measure.vector();
            return err;
        }
    };

    class Pose3d_Plane3d_Factor : public FactorT<Plane3dMeasurement> {
        Pose3d_Node* _pose;
        Plane3d_Node* _surf;
        
    public:
        Pose3d_Plane3d_Factor (Pose3d_Node *pose, Plane3d_Node *surf, 
                               const Plane3dMeasurement& measure, const Noise& noise, Pose3d *senxform1 = NULL, Pose3d *senxform2 = NULL)
            : FactorT<Plane3dMeasurement> ("Pose3d_Plane3d_Factor", Plane3dMeasurement::dim, noise, measure), _pose (pose), _surf (surf) {
            _nodes.resize(2);
            _nodes[0] = pose;
            _nodes[1] = surf;
        }

        void initialize () {
            require(_pose->initialized (), "Pose3d_Plane3d_Factor requires pose to be initialized");
            if (!_surf->initialized ()) {
                
                /* rotate the measured normal to the world frame.  Equivalently, transform
                 * a homogeneous 3d point (with last coordinate 0 instead of 1) whose
                 * x,y,z values are given by the measured normal's x,y,z values */
                const Pose3d& pose = _pose->value ();
                /* HACK: treat norm as homogeneous 3d point w/ w=0. */
                const Point3dh normLocal (_measure.nx (), _measure.ny (), _measure.nz (), 0.);
                const Point3dh normWorldH = pose.transform_from (normLocal);
                Norm3d normWorld (normWorldH.x (), normWorldH.y (), normWorldH.z ());

                /* compute a 3D point that lies on the surface by multipling norm by distance */
                Eigen::VectorXd tmp (3);
                tmp(0) = normWorld.nx ();
                tmp(1) = normWorld.ny ();
                tmp(2) = normWorld.nz ();
                tmp = tmp * _measure.d ();
                Point3d pointLocal (tmp);
                Point3d pointWorld = pose.transform_from (pointLocal);
                Plane3d predict (normWorld.nx (), normWorld.ny (), normWorld.nz (),
                                pointWorld.x (), pointWorld.y (), pointWorld.z ());

                _surf->init (predict);

            }
        }

        Eigen::VectorXd basic_error (Selector s = ESTIMATE) const {
            const Pose3d& pose = _pose->value (s);
            const Plane3d& surf = _surf->value (s);

            /* transform the norm to the local frame */
            /* HACK: treat norm as homogeneous 3d point w/ w=0. */
            const Point3dh normWorld (surf.nx (), surf.ny (), surf.nz (), 0.);
            const Point3dh normLocal = pose.transform_to (normWorld);
            Eigen::Vector3d normPredict (normLocal.x (), normLocal.y (), normLocal.z ());

            /* compute the orthogonal distance of the local frame to surface */
            Eigen::Vector3d p = surf.point ().vector ();
            Eigen::Vector3d t (pose.x (), pose.y(), pose.z());
            Eigen::Vector3d v = p - t;
            double d = normPredict.dot (v);

            Plane3dMeasurement predictMeas (normPredict (0), normPredict (1), normPredict (2), d);

            return (predictMeas.vector () - _measure.vector ());
        }

    };

} // namespace isam
