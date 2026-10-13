# Simulateur de bras robotique

Simulateur de bras robotique développé en **C++17**, utilisant les bibliothèques **Pinocchio** et **Eigen** pour la modélisation du bras et de ses articulations.

Projet réalisé dans le cadre d'un projet C++ (voir `Projet C++.pdf` pour le sujet).

## Description

Ce projet implémente en C++17 la modélisation d'un **bras robotique articulé**. Il s'appuie sur la bibliothèque [Pinocchio](https://github.com/stack-of-tasks/pinocchio) pour la dynamique et la cinématique des corps rigides, et sur [Eigen](https://eigen.tuxfamily.org) pour le calcul matriciel.

Le code est organisé autour de deux notions principales :

- **Les articulations (joints)** : chaque articulation est modélisée avec ses propriétés (type, limites, position courante). 
- **Le bras** : il regroupe plusieurs articulations et permet de manipuler l'ensemble de la chaîne cinématique. 

Le programme principal charge la description du robot à partir d'un fichier **URDF** (format standard de description de robots, via `pinocchio_parsers`) et simule le comportement du bras. 

Chaque composant est validé par des tests unitaires (GoogleTest) : `test_joints` pour les articulations et `test_bras` pour le bras complet. Le code est documenté avec Doxygen (voir `docs/html/`).

### Fonctionnalités

- Modélisation d'un bras à plusieurs articulations 
- Chargement d'un modèle de robot au format URDF 
- Calculs de cinématique avec Pinocchio et Eigen 
- Tests unitaires automatisés
- Documentation du code générée avec Doxygen

## Structure du projet

```
.
├── include/        # Fichiers d'en-tête (.hpp)
├── src/            # Code source (dont main.cpp)
├── tests/          # Tests unitaires (test_bras, test_joints)
├── bin/            # Exécutables compilés
├── docs/html/      # Documentation générée par Doxygen
├── Doxyfile        # Configuration Doxygen
├── makefile
└── Projet C++.pdf  # Sujet du projet
```

## Prérequis

- Compilateur `g++` compatible C++17
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (avec `pinocchio_parsers`)
- [Eigen3](https://eigen.tuxfamily.org)
- [GoogleTest](https://github.com/google/googletest) (pour les tests)
- `pkg-config`
- [Doxygen](https://www.doxygen.nl) (optionnel, pour régénérer la documentation)

## Compilation

| Commande | Action |
|---|---|
| `make` | Compile le programme principal et les tests |
| `make main` | Compile uniquement le programme principal |
| `make tests` | Compile les deux exécutables de test |
| `make test_bras` | Compile uniquement `test_bras` |
| `make test_joints` | Compile uniquement `test_joints` |
| `make clean` | Supprime le dossier `bin/` |

## Utilisation

Lancer le programme principal :

```bash
make run
```

Ou directement :

```bash
./bin/main
```

## Tests

Les tests unitaires utilisent GoogleTest :

```bash
make run_tests
```

Cela exécute successivement `test_joints` puis `test_bras`.

## Documentation

La documentation du code est générée avec Doxygen. Pour la régénérer :

```bash
doxygen Doxyfile
```

Ouvrir ensuite `docs/html/index.html` dans un navigateur.
