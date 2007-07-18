/**********************************************************************
  BondCentricTool - Bond Centric Manipulation Tool for Avogadro

  Copyright (C) 2007 by Shahzad Ali
  Copyright (C) 2007 by Ross Braithwaite
  Copyright (C) 2007 by James Bunt
  Copyright (C) 2007 by Marcus D. Hanwell
  Copyright (C) 2006,2007 by Benoit Jacob

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

#ifndef __BONDCENTRICTOOL_H
#define __BONDCENTRICTOOL_H

#include <avogadro/glwidget.h>
#include <avogadro/tool.h>

#include <openbabel/mol.h>

#include <QGLWidget>
#include <QObject>
#include <QStringList>
#include <QImage>
#include <QAction>

namespace Avogadro {

  /**
   * @class BondCentricTool
   * @brief Bond Centric Molecule Manipulation Tool
   * @author Shahzad Ali, Ross Braithwaite, James Bunt
   *
   * This class is a molecule manipulation system based on bond-centric
   * design as apposed to points in free space design.  It is based off
   * the NavigationTool class by Marcus D. Hanwell.
   */
  class BondCentricTool : public Tool
  {
    Q_OBJECT

    public:
      //! Constructor
      BondCentricTool(QObject *parent = 0);
      //! Deconstructor
      virtual ~BondCentricTool();

      //! \name Description methods
      //@{
      //! Tool Name (ie Draw)
      virtual QString name() const { return(tr("BondCentric")); }
      //! Tool Description (ie. Draws atoms and bonds)
      virtual QString description() const { return(tr("Bond Centric Manipulation Tool")); }
      //@}

      //! \name Tool Methods
      //@{
      //! \brief Callback methods for ui.actions on the canvas.
      /*!
      */
      virtual QUndoCommand* mousePress(GLWidget *widget, const QMouseEvent *event);
      virtual QUndoCommand* mouseRelease(GLWidget *widget, const QMouseEvent *event);
      virtual QUndoCommand* mouseMove(GLWidget *widget, const QMouseEvent *event);
      virtual QUndoCommand* wheel(GLWidget *widget, const QWheelEvent *event);

      virtual int usefulness() const;

      virtual bool paint(GLWidget *widget);

    protected:
      GLWidget *          m_glwidget;
      Atom *              m_clickedAtom;
      Bond *              m_clickedBond;
      Bond *              m_selectedBond;
      Eigen::Vector3d *   m_referencePoint;
      ToolGroup *         m_toolGroup;
      bool                m_leftButtonPressed;  // rotation
      bool                m_midButtonPressed;   // scale / zoom
      bool                m_rightButtonPressed; // translation
      bool                m_movedSinceButtonPressed;

      QPoint              m_lastDraggingPosition;

      QList<GLHit>        m_hits;

      bool isAtomInBond(Atom *atom, Bond *bond);

      void drawAngleSector(GLWidget *widget, Eigen::Vector3d origin,
                           Eigen::Vector3d direction1, Eigen::Vector3d direction2);

      void drawAtomAngles(GLWidget *widget);
      void drawAngles(GLWidget *widget, Atom *atom, Bond *bond);
      //void drawAngles(GLWidget *widget);
      Eigen::Vector3d* calculateSnapTo(GLWidget *widget, Bond *bond, Eigen::Vector3d *referencePoint, double maximumAngle);
      void drawManipulationRectangle(GLWidget *widget, Bond *bond, Eigen::Vector3d *&referencePoint, double rgb[3]);

      void drawSphere(GLWidget *widget,  const Eigen::Vector3d &center, double radius, float alpha);

      void computeClick(const QPoint& p);
      void zoom( const Eigen::Vector3d &goal, double delta ) const;
      void translate( const Eigen::Vector3d &what, const QPoint &from, const QPoint &to ) const;
      void rotate( const Eigen::Vector3d &center, double deltaX, double deltaY ) const;
      void tilt( const Eigen::Vector3d &center, double delta ) const;

      void connectToolGroup(GLWidget *widget);
      void clearData();
      
    private Q_SLOTS:
      void toolChanged(Tool* tool);
      void moleculeChanged(Molecule* previous, Molecule* next);
      void primitiveRemoved(Primitive* primitive);

  };

  class BondCentricToolFactory : public QObject, public ToolFactory
    {
      Q_OBJECT
      Q_INTERFACES(Avogadro::ToolFactory)

      public:
        Tool *createInstance(QObject *parent = 0) { return new BondCentricTool(parent); }
    };

} // end namespace Avogadro

#endif
