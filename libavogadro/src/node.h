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

#ifndef __NODE_H
#define __NODE_H

#include <QList>
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
        bool m_skeletonVisited;
        bool m_nonSkeletonVisited;
      public:
        //! Constructor
        Node();
        virtual ~Node();
        Atom *atom();
        QList<Node*>* nodes();
        bool isSkeletonVisited();
        bool isNonSkeletonVisited();
        bool isLeaf();
        bool contains(Atom* atom);
        void addNode(Node* node);
        void removeNode(Node* node);
        void setAtom(Atom* atom);
        void setSkeletonVisit(bool visited);
        void setNonSkeletonVisit(bool visited);
        void setLeaf(bool leaf);
  };
}
#endif
