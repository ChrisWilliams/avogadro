/**********************************************************************
  Node - Node for skeleton

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

#include "node.h"
#include <QList>
#include <QObject>
#include <openbabel/mol.h>
#include <avogadro/primitive.h>

using namespace OpenBabel;

namespace Avogadro {

Node::Node()
{
  //m_nodes = new QList<Node>();
}

Node::~Node()
{
}

Atom* Node::atom()
{
  return m_atom;
}

QList<Node*>* Node::nodes()
{
  return &m_nodes;
}

bool Node::isSkeletonVisited()
{
  return m_skeletonVisited;
}

bool Node::isNonSkeletonVisited()
{
  return m_nonSkeletonVisited;
}

bool Node::isLeaf()
{
  return m_leaf;
}

bool Node::contains(Atom* atom)
{
  //"atom" exist in the children and grandchildren... of this node.?
  bool exists = false;
  if (m_atom == atom) 
    return true;

  for (int i = 0; i < m_nodes.size(); i++)
    {
      Node* n = m_nodes.at(i);
      if (n->contains(atom))
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

void Node::setAtom(Atom* atom)
{
  m_atom = atom;
}

void Node::setSkeletonVisit(bool visited)
{
  m_skeletonVisited = visited;
}

void Node::setNonSkeletonVisit(bool visited)
{
  m_nonSkeletonVisited = visited;
}

void Node::setLeaf(bool leaf)
{
  m_leaf = leaf;
}
}
