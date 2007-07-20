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
      //@}

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

      //! \name Construction Plane/Angles Methods
      //@{
      //! \brief Methods used to construct and draw the angle-sectors, the construction plane, and the rotation-sphere

      /**
       * Checks whether a given atom is at either end of a given bond.
       *
       * @param atom The atom that is being examined for membership of the given bond.
       * @param bond The bond that is being examined to see if the given atom is
       *             attached to it.
       *
       * @return True if the given atom is the begin or end atom of the given
       *         bond, false otherwise, or if either of the pointers point to NULL.
       */
      bool isAtomInBond(Atom *atom, Bond *bond);

      /**
       * Draws a sector that shows the angle between two lines from a given origin.
       *
       * @param widget The widget this angle-sector will be drawn on.
       * @param origin The origin around which this angle is being calculated.
       * @param direction1 A vector that defines the line from the given origin
       *                   through this point.
       * @param direction2 A vector that defines the line from the given origin
       *                   through this second point.
       */
      void drawAngleSector(GLWidget *widget, Eigen::Vector3d origin,
                           Eigen::Vector3d direction1, Eigen::Vector3d direction2);

      /**
       * Draws sectors around a given atom representing the angles between neighbouring
       * atoms bonded with this atom.
       *
       * @param widget The widget the angle-sectors will be drawn on.
       * @param atom The atom whose angles are being drawn.
       */
      void drawAtomAngles(GLWidget *widget, Atom *atom);

      /**
       * Draws sectors around a given atom representing the angles between neighbouring
       * atoms bonded with this atom and an atom bonded to this atom by a given bond.
       *
       * @param widget The widget the angle-sectors will be drawn on.
       * @param atom The atom whose angles are being drawn.
       * @param bond The bond attached to the given atom that will be used as a reference
       *             point for all the angles.
       *
       * @pre The given atom must be either the begin or end atom of the given bond.
       */
      void drawAngles(GLWidget *widget, Atom *atom, Bond *bond);

      /**
       * Calculates whether the manipulation plane is close enough to any atoms (that
       * are 1 bond away from either of the atoms attached to the given bond) to
       * 'snap-to' them.
       *
       * NOTE: Any atoms that lie along the same line as the bond are disregarded in
       * the calculations otherwise the plane would always try snap-to them as their
       * angle is 0.
       *
       * @param widget The widget the molecule and construction plane are on.
       * @param bond The bond through which the manipulation plane lies.
       * @param referencePoint The current reference point that defines the manipulation
       *                       plane.
       * @param maximumAngle The maximum angle between the current reference point
       *                     and any atom that determines whether or not the plane is
       *                     close enough to snap-to the atom.
       *
       * @return A vector representing the closest Atom to the manipulation plane, to
       *         be used as the reference point for drawing the plane, if any atom is
       *         close enough.  If no atom is close enough to 'snap-to', NULL is
       *         returned.
       */
      Eigen::Vector3d* calculateSnapTo(GLWidget *widget, Bond *bond, Eigen::Vector3d *referencePoint, double maximumAngle);

      /**
       * Draws a rectangle through a bond that can be used as a construction plane to
       * manipulate the bond itself, or the atoms at either end of the bond.
       *
       * @param widget The widget the rectangle will be drawn on.
       * @param bond The bond through which the rectangle will be drawn.
       * @param referencePoint A point orthagonal to the bond that defines the plane
       *                       the rectangle will be drawn on.
       * @param rgb An array of doubles representing the red/green/blue values of the
       *            color for the rectangle.
       */
      void drawManipulationRectangle(GLWidget *widget, Bond *bond, Eigen::Vector3d *&referencePoint, double rgb[3]);

      /**
       * Draws a sphere of a given radius around a given vector.
       *
       * @param widget The widget the sphere will be drawn on.
       * @param center The center of the sphere.
       * @param radius The radius of the sphere.
       * @param alpha The alpha value that determines the opacity of the sphere.
       */
      void drawSphere(GLWidget *widget, const Eigen::Vector3d &center, double radius, float alpha);
      //@}

      void computeClick(const QPoint& p);
      void zoom( const Eigen::Vector3d &goal, double delta ) const;
      void translate( const Eigen::Vector3d &what, const QPoint &from, const QPoint &to ) const;
      void rotate( const Eigen::Vector3d &center, double deltaX, double deltaY ) const;
      void tilt( const Eigen::Vector3d &center, double delta ) const;

      void connectToolGroup(GLWidget *widget);

      /**
       * Clears any data and frees up any memory that is used by the tool.  This
       * procedure should be used when the tool is changed, the molecule cleared,
       * or the program exits etc.
       */
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
