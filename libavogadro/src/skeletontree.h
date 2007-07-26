/**********************************************************************
  SkeletonTree - Tree to make skeleton

  Copyright (C) 2007 by Shahzad Ali
  Copyright (C) 2007 by Ross Braithwaite
  Copyright (C) 2007 by James Bunt

  This file is part of the Avogadro molecular editor project.
  For more information, see <http://avogadro.sourceforge.net/>

  Avogadro is free software; you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation; either version 2 of the License, or 
  (at your option) any later version.

  Avogadro is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301, USA.
 **********************************************************************/

#ifndef __SKELETONTREE_H
#define __SKELETONTREE_H

#include <QObject>
#include <openbabel/mol.h>
#include <avogadro/primitive.h>


namespace Avogadro {

  class Node : public QObject
  {
    protected:
      Atom* m_atom;
      QList<Node*> m_nodes;
      bool m_leaf;
      //bool m_skeletonVisited;
      //bool m_nonSkeletonVisited;

    public:
      //! Constructor
      Node(Atom *atom);
      virtual ~Node();

      Atom *atom();
      QList<Node*> *nodes();

      //bool isSkeletonVisited();
      //bool isNonSkeletonVisited();

      bool isLeaf();
      bool containsAtom(Atom* atom);

      void addNode(Node* node);
      void removeNode(Node* node);

      //void setSkeletonVisit(bool visited);
      //void setNonSkeletonVisit(bool visited);
      void setLeaf(bool leaf);
  };

  /**
   * @class SkeletonTree
   * @brief Skeletal Manipulations on a Molecule
   * @author Shahzad Ali, Ross Braithwaite, James Bunt
   *
   * This class creates and provides methods to manipulate molecule using
   * skeletons.
   */
  class SkeletonTree : public QObject
  {
      public:
        //! Constructor
        SkeletonTree();
        //! Deconstructor
        virtual ~SkeletonTree();

      /**
       * Returns the root node atom.
       *
       * @return The root node atom at which the tree is made.
       */
        Atom *rootAtom();

        Bond *rootBond();

      /**
       * Populates the tree from the molecule, using the root node atom.
       *
       * @param rootAtom The root node atom.
       * @param rootBond The bond at which the root node atom is.
       * @param molecule The molecule to make the tree.
       */
        void populate(Atom *rootAtom, Bond *rootBond, Molecule* molecule);

      /**
       * Translates the atoms attached to root node skeleton, to this location.
       *
       * @param dx The distance the skeleton should move in the x direction.
       * @param dy The distance the skeleton should move in the y direction.
       * @param dz The distance the skeleton should move in the z direction.
       */
        void skeletonTranslate(double dx, double dy, double dz);

      /**
       * Recusively prints the children of this node and child nodes.
       *
       * @param n The node to print.
       */
        void printSkeleton(Node* n);

        bool containsAtom(Atom *atom);

      protected:
        Node *m_rootNode; //The root node, tree
        Bond *m_rootBond; //The bond at which root node atom is attached
        Node *m_endNode;  //A temporary tree.

      private:
        void recursivePopulate(Molecule* mol, Node* node, Bond* bond);
        void recursiveTranslate(Node* n, double x, double y, double z);
  };
}
#endif
