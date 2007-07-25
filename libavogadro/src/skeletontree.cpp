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

#include "skeletontree.h"

using namespace std;

namespace Avogadro {

SkeletonTree::SkeletonTree() {}

SkeletonTree::~SkeletonTree() 
{
  delete m_rootNode;
}

Atom* SkeletonTree::root()
{
  return m_rootNode->atom();
}

// ##########  setRoot  ##########
void SkeletonTree:: setRoot(Atom* atom)
{
  if (!m_rootNode)
  {
    delete m_rootNode;
  }
  m_rootNode = new Node();
  m_rootNode->setAtom(atom);
}

// ##########  setRootBond  ##########
void SkeletonTree:: setRootBond(Bond* bond)
{
  m_rootBond = bond;
}

// ##########  populate  ##########
void SkeletonTree::populate(Molecule* mol)
{
  m_rootNode -> setNonSkeletonVisit(true);
  Atom* bAtom = static_cast<Atom*>(m_rootBond->GetBeginAtom());
  Atom* eAtom = static_cast<Atom*>(m_rootBond->GetEndAtom());
  Atom* diffAtom = (bAtom == m_rootNode->atom()) ? eAtom : bAtom;

  //A temproray tree to find loops
  m_endNode = new Node();
  m_endNode->setAtom(diffAtom);

  //Recursively go through molecule and make a temproray tree.
  //starting from m_endNode
  recursivePopulate(mol, m_endNode, m_rootBond);

  //Recursively go through molecule and make the tree.
  //starting from m_rootNode
  recursivePopulate(mol, m_rootNode, m_rootBond);

  //delete the temporary tree
  delete m_endNode;

  //for debugging puposes
  //printSkeleton(m_rootNode);
}

// ##########  recursivePopulate  ##########
void SkeletonTree::recursivePopulate(Molecule* mol, Node* node, Bond* bond)
{
  Atom* atom = node->atom();
  int found = 0;

  for (unsigned int i=0; i < mol->NumBonds(); i++)
    {
      Bond* b = static_cast<Bond*>(mol->GetBond(i));
      Atom* bAtom = static_cast<Atom*>(b->GetBeginAtom());
      Atom* eAtom = static_cast<Atom*>(b->GetEndAtom());
      if ((b != bond) && ((bAtom == atom) || (eAtom == atom)))
      {
        Atom* diffAtom = (bAtom == atom) ? eAtom : bAtom;

        //Check if this atom already exists, so not to form loops
        if ((!m_endNode->contains(diffAtom)) &&
           (!m_rootNode->contains(diffAtom)))
        {
          Node* newNode = new Node();
          newNode -> setAtom(diffAtom);
          newNode -> setNonSkeletonVisit(true);
          node -> addNode(newNode);
          found++;
          recursivePopulate(mol, newNode, b);
        }
      }
    }
  if (found == 0)
   node->setLeaf(true);
}

// ##########  skeletonTranslate  ##########
void SkeletonTree::skeletonTranslate(double dx, double dy, double dz)
{
  //Translate skeleton
  Atom* a = m_rootNode->atom();
  double dX = dx - a->x();
  double dY = dy - a->y();
  double dZ = dz - a->z();
  recursiveTranslate(m_rootNode, dX, dY, dZ);
}

// ##########  recursiveTranslate  ##########
void SkeletonTree::recursiveTranslate(Node* n, double x, double y, double z)
{
  QList<Node*>* listNodes = n->nodes();
  Atom* a = n->atom();
  a->SetVector(a->x() + x, a->y() + y, a->z() + z);
  for (int i = 0; i < listNodes->size(); i++)
    {
      Node* node = listNodes->at(i);
      recursiveTranslate(node, x, y, z);
    }
}

// ##########  printSkeleton  ##########
void SkeletonTree::printSkeleton(Node* n)
{
  QList<Node*>* listNodes = n->nodes();
  for (int i = 0; i < listNodes->size(); i++)
    {
      Node* n = listNodes->at(i);
      printSkeleton(n);
    }
  Atom* a = n->atom();
  cout << a->x() << "," << a->y()<< ","<<a->z() << endl;
  if (!n->isLeaf())
    cout << "-------------" << endl;
}

}
