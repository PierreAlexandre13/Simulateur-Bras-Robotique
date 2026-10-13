#include <iostream>
#include <memory>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <stdexcept>
#include <string>

#include "../include/Cjoint.hpp"
#include "../include/Cbras.hpp"
#include "../include/Cvecteur.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>





// Compilation :
// g++ -std=c++17 $(pkg-config --cflags --libs pinocchio eigen3) main.cpp -o main

// valgrind --leak-check=full --track-origins=yes ./main



// affiche la position de l'effecteur sur une ligne 
static void printEffector(const CBras& bras, double theta_deg)
{
    Eigen::Matrix4d T = bras.computeFK();
    Eigen::Vector3d p = T.block<3,1>(0,3); //.block<NbLignes, NbColonnes>(ligne_debut, colonne_debut)

    std::cout << std::fixed << std::setprecision(4)
              << "  theta_epaule = " << std::setw(8) << theta_deg << " deg"
              << "  =>  effecteur = ("
              << std::setw(8) << p(0) << ", "
              << std::setw(8) << p(1) << ", "
              << std::setw(8) << p(2) << ")\n";
}


int main()
{   
    const double tol = 1e-10;


    //   Test CJointRevolute
    CJointRevolute joint(-M_PI, M_PI, 0, 0.5);
    joint.setQ(M_PI/4);

    std::cout << "Type: " << joint.getTypeName() << std::endl;
    std::cout << "q = " << joint.getQ() << std::endl;

    Mat4 M = joint.getTransform();
    std::cout << "Transform from class:\n" << M << std::endl;

    //   Validation avec Pinocchio
    Eigen::Matrix3d R;
    R << std::cos(M_PI/4), -std::sin(M_PI/4), 0,
         std::sin(M_PI/4),  std::cos(M_PI/4), 0,
         0,                 0,                1;

    Eigen::Vector3d t(0.5,0,0);
    pinocchio::SE3 T(R,t);
    Eigen::Matrix4d Mref = T.toHomogeneousMatrix();

    std::cout << "Transform from pinocchio:\n" << Mref << std::endl;

    assert(M.isApprox(Mref, tol));
    std::cout << "Validation getTransform() OK\n";


    //   Test clone()
    auto jclone = joint.clone();
    std::cout << "Clone type: " << jclone->getTypeName() << std::endl;
    Mat4 Mclone = jclone->getTransform();

    assert(M.isApprox(Mclone, tol));
    std::cout << "Clone OK\n\n";



    //############################


    CBras bras2;
    bras2.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.5));
    bras2.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.3, 0.3));
    bras2.addJoint(std::make_unique<CJointPrismatic>(0.0, 0.5, 0.1));

    std::cout << bras2;
    // Bras [3 DDL]
    // [0] Revolute  q = 0.000 rad bornes=[-3.142,  3.142]
    // [1] Revolute  q = 0.300 rad bornes=[-3.142,  3.142]
    // [2] Prismatic q = 0.100 m   bornes=[ 0.000,  0.500]

    std::cout << "\nMatrice FK :\n" << bras2.computeFK() << "\n";

    Eigen::VectorXd q = bras2.getQ();
    std::cout << "q = " << q << std::endl;

    q = bras2.randomQ();
    std::cout << "random q = " << q << std::endl;

    Eigen::Vector3d q2(1, 2, 0.4);
    bras2.setQ(q2);

    q = bras2.getQ();
    std::cout << "q = " << q << "\n\n";


    //##########################

    std::cout << "=== Tests CVecteur<double, 3> ===\n\n";

    // ── Construction ───
    std::cout << "-- Constructeurs --\n";

    CVecteur<double, 3> zero;                    // par défaut : {0, 0, 0}
    CVecteur<double, 3> a = {1.0, 2.0, 3.0};
    CVecteur<double, 3> b = {4.0, 5.0, 6.0};

    std::cout << "zero : " << zero << std::endl;
    std::cout << "a : " << a << std::endl;
    std::cout << "b : " << b << std::endl;

    // ── operator[] ───
    std::cout << "\n-- operator[] --\n";

    a[0] = 10.0;
    std::cout << "10 : " << a[0] << std::endl;
    a[0] = 1.0;                                  // remise à l'état initial


    // ── Opérations arithmétiques ───
    std::cout << "\n-- Opérations --\n";

    CVecteur<double, 3> somme = a + b;           // {5, 7, 9}
    std::cout << "somme : " << somme << std::endl;

    CVecteur<double, 3> diff = b - a;            // {3, 3, 3}
    std::cout << "diff : " << diff << std::endl;

    CVecteur<double, 3> scaled = a * 3.0;        // {3, 6, 9}
    std::cout << "*3 : " << scaled << std::endl;
    
    double d = a.dot(b);                         // 1*4 + 2*5 + 3*6 = 32
    std::cout << "dot 32 : " << d << std::endl;

    CVecteur<double, 3> unit = {1.0, 0.0, 0.0};
    std::cout << "norme 1 : " << unit.norm() << std::endl;

    CVecteur<double, 3> v345 = {3.0, 4.0, 0.0};
    std::cout << "norme 5 : " << v345.norm() << std::endl;

    Eigen::Matrix<double, 3, 1> ea = a.toEigen();
    std::cout << "to Eigen : " << ea << std::endl;

    // Vérifie que c'est bien une copie indépendante (zéro partage mémoire)
    a[0] = 99.0;
    std::cout << "inde != 99 : " << ea[0] << std::endl;
    a[0] = 1.0;

    CVecteur<double, 3> u = {1.0, 0.0, 0.0};
    CVecteur<double, 3> v = {0.0, 1.0, 0.0};
    std::cout << "dot 0 : " << u.dot(v) << std::endl;
    std::cout << "norme 1 : " << u.norm() << "\n\n";


    //#######################


    // Construction du bras 4-DDL
    //  Joint     Type        Limites           dx
    //  Epaule    Revolute    [-π,   π]         0.30 m
    //  Coude     Revolute    [-π/2, π/2]       0.25 m
    //  Ext.      Prismatic   [0,    0.20]      –
    //  Poignet   Revolute    [-π,   π]         0.05 m

    CBras bras;
    bras.addJoint(std::make_unique<CJointRevolute> (-M_PI,   M_PI,   0.0, 0.30)); 
    bras.addJoint(std::make_unique<CJointRevolute> (-M_PI/2, M_PI/2, 0.0, 0.25)); 
    bras.addJoint(std::make_unique<CJointPrismatic>(0.0,     0.20,   0.0      )); 
    bras.addJoint(std::make_unique<CJointRevolute> (-M_PI,   M_PI,   0.0, 0.05)); 

    std::cout << bras << "\n";


    std::cout << "── Balayage épaule [-π/2 .. π/2] pas π/8 ──────────────────\n";

    for (int k = -4; k <= 4; ++k)
    {
        double theta = k * M_PI / 8.0;          // de -π/2 à π/2
        bras.getJoint(0).setQ(theta);
        printEffector(bras, theta * 180.0 / M_PI);
    }
    bras.getJoint(0).setQ(0.0);                 // remise à zéro
    std::cout << "\n";



    std::cout << "── Test std::out_of_range sur setQ() ───────────────────────\n";
    try {
        // Coude limité à [-π/2, π/2] : π dépasse la butée haute
        bras.getJoint(1).setQ(M_PI);
        std::cout << "  ERREUR : exception non levée !\n";
    }
    catch (const std::out_of_range& e) {
        std::cout << "  Exception capturée (setQ hors bornes)  : " << e.what() << "\n";
    }

    try {
        // Accès à un joint inexistant (N = 4, indice 10)
        bras.getJoint(10);
        std::cout << "  ERREUR : exception non levée !\n";
    }
    catch (const std::out_of_range& e) {
        std::cout << "  Exception capturée (getJoint hors limites) : " << e.what() << "\n";
    }
    std::cout << "\n";



    std::cout << "── Test copie profonde ──────────────────────────────────────\n";

    CBras copie = bras;                          // constructeur de copie (clone)

    // FK de la copie avant toute modification
    Eigen::Matrix4d T_avant = copie.computeFK();

    // Modification de l'original uniquement (épaule à π/4)
    bras.getJoint(0).setQ(M_PI / 4.0);
    Eigen::Matrix4d T_original = bras.computeFK();

    // La copie doit refléter l'état d'avant la modification
    Eigen::Matrix4d T_copie = copie.computeFK();

    bool copie_ok = true;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::abs(T_avant(i,j) - T_copie(i,j)) > tol)
                copie_ok = false;

    std::cout << "  FK original (épaule = π/4) :\n" << T_original << "\n";
    std::cout << "  FK copie    (épaule = 0)   :\n" << T_copie    << "\n";
    std::cout << "  Copie indépendante de l'original : " << (copie_ok ? "OK " : "ECHEC ") << "\n\n";

    bras.getJoint(0).setQ(0.0);                 // remise à zéro


    
    // std::move : l'original doit être vide après déplacement
    std::cout << "── Test std::move ───────────────────────────────────────────\n";

    std::size_t nb_avant = bras.getNbJoints();
    std::cout << "  Nb joints avant move (original) : " << nb_avant << "\n";

    CBras bras_deplace = std::move(bras);        // move constructor

    std::cout << "  Nb joints après move (original) : " << bras.getNbJoints()
              << (bras.getNbJoints() == 0 ? "  -> OK (vide)" : "  -> ECHEC")   << "\n";
    std::cout << "  Nb joints dans bras_deplace     : " << bras_deplace.getNbJoints()
              << (bras_deplace.getNbJoints() == nb_avant ? "  -> OK (" + std::to_string(nb_avant) + " DDL)" : "  -> ECHEC") << "\n\n";

    std::cout << bras_deplace;



    //############################


    // Validation Pinocchio UR5
    std::cout << "── Validation Pinocchio UR5 ─────────────────────────────────\n";
 
    // 6a. Chargement du modèle URDF
    pinocchio::Model model;
    
    static const std::string URDF_PATH = "/opt/openrobots/share/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf";
    pinocchio::urdf::buildModel(URDF_PATH, model);
   
    pinocchio::Data data(model);
  
    std::cout << "  Modèle UR5 chargé : " << model.nq << " DDL, " << model.njoints - 1 << " joints mobiles\n\n";
 
    // ── 6b. Configuration q0 = 0.5 rad sur joint 1, autres à 0 ─────────────
    // q est un vecteur de taille model.nq (7 pour UR5 avec les quaternions
    // du joint flottant racine, ou 6 si le joint racine est fixe)
    Eigen::VectorXd q3 = Eigen::VectorXd::Zero(model.nq);
    q3(0) = 0.5;   // premier degré de liberté (épaule UR5)
 
    // Cinématique directe Pinocchio
    pinocchio::forwardKinematics(model, data, q3);
 
    // Transformation du 1er joint (indice 1 dans oMi)
    Eigen::Matrix4d T_pinocchio = data.oMi[1].toHomogeneousMatrix();
 
    // ── 6c. Même configuration dans CBras (joint revolute, q=0.5, dx=0) ────
    // On reconstruit un bras minimaliste avec un seul revolute pour comparer
    // le premier joint UR5 (rotation pure, pas de translation de lien dans
    // oMi[1] qui est la pose du joint, pas de l'effecteur)
    CBras bras_ur5_j1;
    bras_ur5_j1.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.5, 0.0));
    bras_ur5_j1.addJoint(std::make_unique<CJointPrismatic>(0, 1, 0.089159));
    Eigen::Matrix4d T_cbras = bras_ur5_j1.computeFK();
 
    double err = (T_pinocchio - T_cbras).norm() ; //frobeniusError(T_pinocchio, T_cbras)   
 
    std::cout << "  T Pinocchio oMi[1] (q0=0.5) :\n" << std::fixed << std::setprecision(6) << T_pinocchio << "\n";
    std::cout << "  T CBras     (q0=0.5, dx=0)  :\n"  << T_cbras << "\n";
    std::cout << "  Erreur de Frobenius : " << std::scientific << err << "\n";
 
    // ── 6d. Configuration aléatoire (démonstration forwardKinematics) ───────
    std::cout << "── Configuration aléatoire Pinocchio ────────────────────────\n";
    Eigen::VectorXd q_rand = pinocchio::randomConfiguration(model);
    pinocchio::forwardKinematics(model, data, q_rand);
 
    std::cout << "  q_rand = " << q_rand.transpose() << "\n";
    std::cout << "  Pose effecteur (dernier joint) :\n" << data.oMi[model.njoints - 1].toHomogeneousMatrix() << "\n\n";
 



    // ═══════════════════════════════════════════════════════════════════════
    // Régulateur proportionnel 2D
    //    Cible (x*, y*), erreur e = (x* - x, y* - y)
    //    Δθ = Kp * ||e||   (Kp = 0.5)
    //    Arrêt quand ||e|| < 0.01 m
    // ═══════════════════════════════════════════════════════════════════════
    std::cout << "── Régulateur proportionnel (P-controller 2D) ───────────────\n";
 
    const double Kp       = 0.5;
    const double seuil    = 0.01;   // m
    const int    max_iter = 200;
 
    // Cible
    const Eigen::Vector2d cible(0.30, 0.30); 
 
    // État initial : épaule à 0, on lit la position (x,y) du CBras déplacé
    // (on repart de bras_deplace dont tous les joints sont à 0)
    double theta_epaule = 0.0;
 
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Cible : (" << cible(0) << ", " << cible(1) << ") m\n\n";
    std::cout << "  iter    theta(rad)    x        y        ||e||\n";
    std::cout << "  ──────────────────────────────────────────────\n";
 
    for (int iter = 0; iter < max_iter; ++iter)
    {
        // Position actuelle de l'effecteur (x, y) via computeFK()
        bras_deplace.getJoint(0).setQ(theta_epaule); //bras_ur5_j1 
        Eigen::Matrix4d Tk = bras_deplace.computeFK();
        Eigen::Vector2d pos(Tk(0,3), Tk(1,3));
 
        // Erreur
        Eigen::Vector2d e = cible - pos;
        double norme_e = e.norm();
 
        // Affichage toutes les 5 itérations + première + dernière convergée
        if (iter % 5 == 0 || norme_e < seuil)
            std::cout << "  " << std::setw(4) << iter
                      << "    " << std::setw(9) << theta_epaule
                      << "    " << std::setw(7) << pos(0)
                      << "    " << std::setw(7) << pos(1)
                      << "    " << std::setw(7) << norme_e << "\n";
 
        // Critère d'arrêt
        if (norme_e < seuil) {
            std::cout << "\n  Convergé en " << iter << " itérations. "
                      << "||e|| = " << norme_e << " m  < seuil " << seuil << " m \n";
            break;
        }
 
        // Loi de commande proportionnelle
        double delta_theta = Kp * norme_e;
 
        // On sature delta_theta pour rester dans les limites du joint
        double q_new = theta_epaule + delta_theta;
        q_new = std::max(-M_PI, std::min(M_PI, q_new));
        theta_epaule = q_new;
 
        if (iter == max_iter - 1)
            std::cout << "\n  Non convergé après " << max_iter << " itérations.\n";
    }
 



    return 0;
}