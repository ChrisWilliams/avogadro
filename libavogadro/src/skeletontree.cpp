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
using namespace Avogadro;
using namespace Eigen;

// ################################## Node #####################################

Node::Node(Atom *atom)
{
  m_atom = atom;
  //m_nodes = new QList<Node>();
}

Node::~Node() {}

Atom* Node::atom()
{
  return m_atom;
}

QList<Node*> *Node::nodes()
{
  return &m_nodes;
}

/*bool Node::isSkeletonVisited()
{
  return m_skeletonVisited;
}

bool Node::isNonSkeletonVisited()
{
  return m_nonSkeletonVisited;
}*/

bool Node::isLeaf()
{
  return m_leaf;
}

bool Node::containsAtom(Atom* atom)
{
  //"atom" exist in the children and grandchildren... of this node.?
  bool exists = false;
  if (m_atom == atom) 
    return true;

  for (int i = 0; i < m_nodes.size(); i++)
  {
    Node* n = m_nodes.at(i);
    if (n->containsAtom(atom))
    {
      exists = true;
      break;
    }
  }
  return exists;
}

void Node::addNode(Node* node)
{
  m_nodes.append(node);
}

void Node::removeNode(Node* node)
{
  int i = m_nodes.indexOf(node);
  if (i != -1)
  {
    m_nodes.removeAt(i);
  }
}

/*void Node::setSkeletonVisit(bool visited)
{
  m_skeletonVisited = visited;
}

void Node::setNonSkeletonVisit(bool visited)
{
  m_nonSkeletonVisited = visited;
}*/

void Node::setLeaf(bool leaf)
{
  m_leaf = leaf;
}

// ############################## SkeletonTree #################################

SkeletonTree::SkeletonTree() {}

SkeletonTree::~SkeletonTree() 
{
  delete m_rootNode;
}

Atom* SkeletonTree::rootAtom()
{
  return m_rootNode->atom();
}

Bond* SkeletonTree::rootBond()
{
  return m_rootBond;
}
// ##########  populate  ##########
void SkeletonTree::populate(Atom *rootAtom, Bond *rootBond, Molecule* molecule)
{
  if (!m_rootNode)
  {
    delete m_rootNode;
  }

  m_rootNode = new Node(rootAtom);

  m_rootBond = rootBond;

  Atom* bAtom = static_cast<Atom*>(m_rootBond->GetBeginAtom());
  Atom* eAtom = static_cast<Atom*>(m_rootBond->GetEndAtom());

  if (bAtom != m_rootNode->atom() && eAtom != m_rootNode->atom())
    return;

  Atom* diffAtom = (bAtom == m_rootNode->atom()) ? eAtom : bAtom;

  //A temproray tree to find loops
  m_endNode = new Node(diffAtom);

  //Recursively go through molecule and make a temproray tree.
  //starting from m_endNode
  recursivePopulate(molecule, m_endNode, m_rootBond);

  //Recursively go through molecule and make the tree.
  //starting from m_rootNode
  recursivePopulate(molecule, m_rootNode, m_rootBond);

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
        if ((!m_endNode->containsAtom(diffAtom)) &&
           (!m_rootNode->containsAtom(diffAtom)))
        {
          Node* newNode = new Node(diffAtom);
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
  if (m_rootNode) {
    //Translate skeleton
    recursiveTranslate(m_rootNode, dx, dy, dz);
  }
}

// ##########  skeletonRotate  ##########
void SkeletonTree::skeletonRotate(double angle, Eigen::Vector3d rotationVector, Eigen::Vector3d centerVector)
{
  if (m_rootNode) {
    //Rotate skeleton
    recursiveRotate(m_rootNode, angle, rotationVector, centerVector);
  }
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

// ##########  recursiveRotate  ##########
void SkeletonTree::recursiveRotate(Node* n, double angle, Eigen::Vector3d rotationVector, Eigen::Vector3d centerVector)
{
  QList<Node*>* listNodes = n->nodes();
  Atom* a = n->atom();
  Vector3d final = performRotation(angle, rotationVector, centerVector, a->pos());
  a->SetVector(final.x(), final.y(), final.z());
  for (int i = 0; i < listNodes->size(); i++)
  {
    Node* node = listNodes->at(i);
    recursiveRotate(node, angle, rotationVector, centerVector);
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

bool SkeletonTree::containsAtom(Atom *atom)
{
  return m_rootNode ? m_rootNode->containsAtom(atom) : false;
}

Eigen::Vector3d SkeletonTree::performRotation(double angle, Eigen::Vector3d rotationVector, Eigen::Vector3d centerVector, Eigen::Vector3d positionVector)
{
  double angleHalf = angle/2.0;
  double rotationQw = cos(angleHalf);
  double sinAngle = sin(angleHalf);
  Vector3d rotationQv = Vector3d(rotationVector.x() * sinAngle, rotationVector.y()
                                 * sinAngle, rotationVector.z() * sinAngle);
  double directionQw = 0;
  Vector3d directionQv = positionVector - centerVector;
  double tempQw = rotationQw * directionQw - rotationQv.dot(directionQv);
  Vector3d tempQv = Vector3d(rotationQw*directionQv.x() + rotationQv.x()*directionQw     + rotationQv.y()*directionQv.z() - rotationQv.z()*directionQv.y(),
                              rotationQw*directionQv.y() - rotationQv.x()*directionQv.z() + rotationQv.y()*directionQw     + rotationQv.z()*directionQv.x(),
                              rotationQw*directionQv.z() + rotationQv.x()*directionQv.y() - rotationQv.y()*directionQv.x() + rotationQv.z()*directionQw     );
  double rotationQnorm2 = (rotationQw*rotationQw+rotationQv.norm2());
  double rotationQwINV = rotationQw / rotationQnorm2;
  Vector3d rotationQvINV = - rotationQv / rotationQnorm2;
  //double finalQw = tempQw * rotationQwINV - tempQv.dot(rotationQvINV);
  Vector3d finalQv = Vector3d(tempQw*rotationQvINV.x() + tempQv.x()*rotationQwINV     + tempQv.y()*rotationQvINV.z() - tempQv.z()*rotationQvINV.y(),
                              tempQw*rotationQvINV.y() - tempQv.x()*rotationQvINV.z() + tempQv.y()*rotationQwINV     + tempQv.z()*rotationQvINV.x(),
                              tempQw*rotationQvINV.z() + tempQv.x()*rotationQvINV.y() - tempQv.y()*rotationQvINV.x() + tempQv.z()*rotationQwINV     );
  return finalQv + centerVector;
}
