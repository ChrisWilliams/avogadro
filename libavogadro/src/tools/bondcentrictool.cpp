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

#include "bondcentrictool.h"
#include <avogadro/primitive.h>
#include <avogadro/color.h>
#include <avogadro/glwidget.h>
#include <avogadro/camera.h>
#include <avogadro/toolgroup.h>

#include <openbabel/obiter.h>
#include <openbabel/mol.h>

#include <QtPlugin>
#include <QString>

using namespace std;
using namespace OpenBabel;
using namespace Avogadro;
using namespace Eigen;

// ##########  Constructor  ##########

BondCentricTool::BondCentricTool(QObject *parent) : Tool(parent),
                                                    m_clickedAtom(NULL),
                                                    m_clickedBond(NULL),
                                                    m_selectedBond(NULL),
                                                    m_referencePoint(NULL),
                                                    m_toolGroup(NULL),
                                                    m_leftButtonPressed(false),
                                                    m_midButtonPressed(false),
                                                    m_rightButtonPressed(false),
                                                    m_movedSinceButtonPressed(false)
{
  QAction *action = activateAction();
  action->setIcon(QIcon(QString::fromUtf8(":/bondcentric/bondcentric.png")));
  action->setToolTip(tr("Bond Centric Manipulation Tool\n\n"
        "Left Mouse:   Click and drag to rotate the view\n"
        "Middle Mouse: Click and drag to zoom in or out\n"
        "Right Mouse:  Click and drag to move the view"));
  GLubyte m1[] = { 0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55 };
    GLubyte m2[] = { 0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55,
          0xAA, 0xAA, 0xAA, 0xAA,
          0x55, 0x55, 0x55, 0x55, 
          0xAA, 0xAA, 0xAA, 0xAA };
    GLubyte m3[] = { 0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF,
          0xFF, 0xFF, 0xFF, 0xFF, 
          0xFF, 0xFF, 0xFF, 0xFF };
    mask1 = new GLubyte[128];
    mask2 = new GLubyte[128];
    mask3 = new GLubyte[128];
    for(int i = 0; i < 128; i++)
    {
      mask1[i] = m1[i];
      mask2[i] = m2[i];
      mask3[i] = m3[i];
    }
  //action->setShortcut(Qt::Key_F9);
}

// ##########  Desctructor  ##########

BondCentricTool::~BondCentricTool()
{
  delete m_referencePoint;
  m_referencePoint = NULL;
  delete[] mask1;
  delete[] mask2;
  delete[] mask3;
}

// ##########  clearData  ##########

void BondCentricTool::clearData()
{
  m_clickedAtom = NULL;
  m_clickedBond = NULL;
  m_selectedBond = NULL;
  delete m_referencePoint;
  m_referencePoint = NULL;
  m_toolGroup = NULL;
  m_leftButtonPressed = false;
  m_midButtonPressed = false;
  m_rightButtonPressed = false;
  m_movedSinceButtonPressed = false;
}

// ##########  moleculeChanged  ##########

void BondCentricTool::moleculeChanged(Molecule* previous, Molecule* next)
{
  if (previous)
  {
    disconnect(previous,0,this,0);
  }
  if (next)
  {
    connect((Primitive*)next,SIGNAL(primitiveRemoved(Primitive*)),this,SLOT(primitiveRemoved(Primitive*)));
  }
  clearData();
}

// ##########  primitiveRemoved  ##########

void BondCentricTool::primitiveRemoved(Primitive *primitive)
{
  if (primitive == m_clickedAtom || primitive == m_clickedBond || primitive == m_selectedBond)
  {
    clearData();
  }
}

// ##########  connectToolGroup  ##########

void BondCentricTool::connectToolGroup(GLWidget *widget)
{
  if(widget->toolGroup() != m_toolGroup && widget->toolGroup())
  {
    disconnect(widget->toolGroup(), 0, this, 0);
    connect(widget->toolGroup(), SIGNAL(toolActivated(Tool*)),
          this, SLOT(toolChanged(Tool*)));
    m_toolGroup = widget->toolGroup();
  }
}

// ##########  toolChanged  ##########

void BondCentricTool::toolChanged(Tool* tool)
{
  if(tool != this && m_glwidget)
  {
    m_glwidget->update();
    clearData();
  }
}

// ##########  usefulness  ##########

int BondCentricTool::usefulness() const
{
  return 2000000;
}

// ##########  computeClick  ##########

void BondCentricTool::computeClick(const QPoint& p)
{
  int oldName = m_selectedBond ? m_selectedBond->GetIdx() : -1;
  m_clickedAtom = NULL;
  m_clickedBond = NULL;

  // Perform a OpenGL selection and retrieve the list of hits.
  m_hits = m_glwidget->hits(p.x()-SEL_BOX_HALF_SIZE,
      p.y()-SEL_BOX_HALF_SIZE,
      SEL_BOX_SIZE, SEL_BOX_SIZE);

  Molecule *molecule = m_glwidget->molecule();

  // Find the first atom (if any) in hits - this will be the closest
  foreach(GLHit hit, m_hits)
  {
    if (hit.type() == Primitive::BondType)
    {
      m_clickedBond = static_cast<Bond *>(molecule->GetBond(hit.name()-1));
      if (m_leftButtonPressed)
      {
        m_selectedBond = m_clickedBond;

        if ((int)m_selectedBond->GetIdx() != oldName) {
          delete m_referencePoint;
          m_referencePoint = NULL;
        }
      }
      return;
    }
    else if (hit.type() == Primitive::AtomType)
    {
      m_clickedAtom = static_cast<Atom *>(molecule->GetAtom(hit.name()));
      return;
    }
  }
}

// ##########  zoom  ##########

void BondCentricTool::zoom( const Eigen::Vector3d &goal, double delta ) const
{
  Vector3d transformedGoal = m_glwidget->camera()->modelview() * goal;
  double distanceToGoal = transformedGoal.norm();

  double t = ZOOM_SPEED * delta;
  const double minDistanceToGoal = 2.0 * CAMERA_NEAR_DISTANCE;
  double u = minDistanceToGoal / distanceToGoal - 1.0;

  if (t < u) {
    t = u;
  }

  m_glwidget->camera()->modelview().pretranslate( transformedGoal * t );
}

// ##########  translate  ##########

void BondCentricTool::translate( const Eigen::Vector3d &what, const QPoint &from, const QPoint &to ) const
{
  Vector3d fromPos = m_glwidget->camera()->unProject(from, what);
  Vector3d toPos = m_glwidget->camera()->unProject(to, what);
  m_glwidget->camera()->translate(toPos - fromPos);
}

// ##########  rotate  ##########

void BondCentricTool::rotate( const Eigen::Vector3d &center, double deltaX, double deltaY ) const
{
  Vector3d xAxis = m_glwidget->camera()->backtransformedXAxis();
  Vector3d yAxis = m_glwidget->camera()->backtransformedYAxis();
  m_glwidget->camera()->translate(center);
  m_glwidget->camera()->rotate(deltaX * ROTATION_SPEED, yAxis);
  m_glwidget->camera()->rotate(deltaY * ROTATION_SPEED, xAxis);
  m_glwidget->camera()->translate(-center);
}

// ##########  tilt  ##########

void BondCentricTool::tilt( const Eigen::Vector3d &center, double delta ) const
{
  Vector3d zAxis = m_glwidget->camera()->backtransformedZAxis();
  m_glwidget->camera()->translate(center);
  m_glwidget->camera()->rotate(delta * ROTATION_SPEED, zAxis);
  m_glwidget->camera()->translate(-center);
}

// ##########  mousePress  ##########

QUndoCommand* BondCentricTool::mousePress(GLWidget *widget, const QMouseEvent *event)
{
  if(m_glwidget != widget)
  {
    disconnect(widget,0,this,0);
    connect(widget,SIGNAL(moleculeChanged(Molecule*,Molecule*)),this,SLOT(moleculeChanged(Molecule*,Molecule*)));
    m_glwidget = widget;
    moleculeChanged(NULL,m_glwidget->molecule());
    connectToolGroup(widget);
  }
 
  m_lastDraggingPosition = event->pos();
  m_movedSinceButtonPressed = false;
#ifdef Q_WS_MAC
  m_leftButtonPressed = (event->buttons() & Qt::LeftButton 
                         && event->modifiers() == Qt::NoModifier);
  // On the Mac, either use a three-button mouse
  // or hold down the Option key (AltModifier in Qt notation)
  m_midButtonPressed = ((event->buttons() & Qt::MidButton) || 
                         (event->buttons() & Qt::LeftButton && event->modifiers() & Qt::AltModifier));
  // Hold down the Command key (ControlModifier in Qt notation) for right button
  m_rightButtonPressed = ((event->buttons() & Qt::RightButton) || 
                           (event->buttons() & Qt::LeftButton && event->modifiers() & Qt::ControlModifier));
#else
  m_leftButtonPressed = (event->buttons() & Qt::LeftButton);
  m_midButtonPressed = (event->buttons() & Qt::MidButton);
  m_rightButtonPressed = (event->buttons() & Qt::RightButton);
#endif
  computeClick(event->pos());

  widget->update();
  return 0;
}

// ##########  mouseRelease  ##########

QUndoCommand* BondCentricTool::mouseRelease(GLWidget *widget, const QMouseEvent*)
{
  if (!m_hits.size() && !m_movedSinceButtonPressed) {
    delete m_referencePoint;
    m_referencePoint = NULL;
    m_selectedBond = NULL;
  }

  m_glwidget = widget;
  m_leftButtonPressed = false;
  m_midButtonPressed = false;
  m_rightButtonPressed = false;
  m_clickedAtom = NULL;
  m_clickedBond = NULL;

  widget->update();
  return 0;
}

// ##########  mouseMove  ##########

QUndoCommand* BondCentricTool::mouseMove(GLWidget *widget, const QMouseEvent *event)
{
  m_glwidget = widget;
  if (!m_glwidget->molecule()) {
    return 0;
  }

  QPoint deltaDragging = event->pos() - m_lastDraggingPosition;

  if ((event->pos() - m_lastDraggingPosition).manhattanLength() > 2)
    m_movedSinceButtonPressed = true;

  // Mouse navigation has two modes - atom centred when an atom is clicked
  // and scene if no atom has been clicked.

#ifdef Q_WS_MAC
  if (event->buttons() & Qt::LeftButton 
      && event->modifiers() == Qt::NoModifier)
#else
  if (event->buttons() & Qt::LeftButton)
#endif
  {
    if (m_clickedBond && m_selectedBond && m_referencePoint)
    {
      Atom *beginAtom = static_cast<Atom*>(m_selectedBond->GetBeginAtom());
      Atom *endAtom = static_cast<Atom*>(m_selectedBond->GetEndAtom());

      Vector3d rotationVector = beginAtom->pos() - endAtom->pos();
      rotationVector = rotationVector / rotationVector.norm();

      Vector3d begin = widget->camera()->project(beginAtom->pos());
      Vector3d end = widget->camera()->project(endAtom->pos());

      Vector3d zAxis = Vector3d(0, 0, 1);
      Vector3d beginToEnd = end - begin;
      beginToEnd -= Vector3d(0, 0, beginToEnd.z());

      Vector3d direction = zAxis.cross(beginToEnd);
      direction = direction / direction.norm();

      Vector3d mouseMoved = Vector3d(deltaDragging.x(), - deltaDragging.y(), 0);

      double magnitude = mouseMoved.dot(direction) / direction.norm();

      Matrix3d rotationMatrix;

      rotationMatrix.loadRotation3((magnitude * (M_PI / 180.0)), rotationVector);
      rotationMatrix.multiply(*m_referencePoint, m_referencePoint);
    }
    else if (isAtomInBond(m_clickedAtom, m_selectedBond))
    {
      //Do atom rotation.
    }
    else
    {
      // rotation around the center of the molecule
      rotate(m_glwidget->center(), deltaDragging.x(), deltaDragging.y());
    }
  }
#ifdef Q_WS_MAC
  // On the Mac, either use a three-button mouse
  // or hold down the Option key (AltModifier in Qt notation)
  else if ((event->buttons() & Qt::MidButton) || 
            (event->buttons() & Qt::LeftButton && event->modifiers() & Qt::AltModifier))
#else
  else if (event->buttons() & Qt::MidButton)
#endif
  {
    if (m_clickedAtom)
    {
      // Perform the rotation
      tilt(m_clickedAtom->pos(), deltaDragging.x());

      // Perform the zoom toward the center of a clicked atom
      zoom(m_clickedAtom->pos(), deltaDragging.y());
    }
    else if (m_clickedBond)
    {
      Atom *begin = static_cast<Atom *>(m_clickedBond->GetBeginAtom());
      Atom *end = static_cast<Atom *>(m_clickedBond->GetEndAtom());

      Vector3d btoe = end->pos() - begin->pos();
      double newLen = btoe.norm() / 2;
      btoe = btoe / btoe.norm();

      Vector3d mid = begin->pos() + btoe * newLen;

      // Perform the rotation
      tilt(mid, deltaDragging.x());

      // Perform the zoom toward the centre of a clicked bond
      zoom(mid, deltaDragging.y());
    }
    else
    {
      // Perform the rotation
      tilt(m_glwidget->center(), deltaDragging.x());

      // Perform the zoom toward molecule center
      zoom(m_glwidget->center(), deltaDragging.y());
    }
  }
#ifdef Q_WS_MAC
  // On the Mac, either use a three-button mouse
  // or hold down the Command key (ControlModifier in Qt notation)
  else if ((event->buttons() & Qt::RightButton) || 
            (event->buttons() & Qt::LeftButton && event->modifiers() & Qt::ControlModifier))
#else
  else if (event->buttons() & Qt::RightButton)
#endif
  {
    if (isAtomInBond(m_clickedAtom, m_selectedBond))
    {
    }
    else
    {
      // translate the molecule following mouse movement
      translate(m_glwidget->center(), m_lastDraggingPosition, event->pos());
    }
  }

  m_lastDraggingPosition = event->pos();
  m_glwidget->update();

  return 0;
}

// ##########  wheel  ##########

QUndoCommand* BondCentricTool::wheel(GLWidget *widget, const QWheelEvent *event)
{
  m_glwidget = widget;
  computeClick(event->pos());

  if (m_clickedAtom)
  {
    // Perform the zoom toward clicked atom
    zoom(m_clickedAtom->pos(), - MOUSE_WHEEL_SPEED * event->delta());
  }
  else if (m_clickedBond)
  {
    Atom *begin = static_cast<Atom *>(m_clickedBond->GetBeginAtom());
    Atom *end = static_cast<Atom *>(m_clickedBond->GetEndAtom());

    Vector3d btoe = end->pos() - begin->pos();
    double newLen = btoe.norm() / 2;
    btoe = btoe / btoe.norm();

    Vector3d mid = begin->pos() + btoe * newLen;

    // Perform the zoom toward the centre of a clicked bond
    zoom(mid, - MOUSE_WHEEL_SPEED * event->delta());
  }
  else
  {
    // Perform the zoom toward molecule center
    zoom(m_glwidget->center(), - MOUSE_WHEEL_SPEED * event->delta());
  }

  m_glwidget->update();

  m_clickedAtom = NULL;
  m_clickedBond = NULL;

  return 0;
}

// ##########  paint  ##########

bool BondCentricTool::paint(GLWidget *widget)
{
  if(widget->toolGroup()->activeTool() != this)
  {
    clearData();
  }

  if ((m_leftButtonPressed && !m_clickedBond && !isAtomInBond(m_clickedAtom, m_selectedBond))
       || (m_midButtonPressed && !m_clickedBond && !m_clickedAtom)
       || (m_rightButtonPressed && !isAtomInBond(m_clickedAtom, m_selectedBond)))
  {
    drawSphere(widget, widget->center(), 0.10, 1.0);
  }

  if (m_leftButtonPressed && m_clickedAtom && (!m_selectedBond ||
      !isAtomInBond(m_clickedAtom, m_selectedBond)))
  {
    drawAtomAngles(widget, mask1);
  }

  if (m_selectedBond)
  {
    Atom *begin = static_cast<Atom*>(m_selectedBond->GetBeginAtom());
    Atom *end = static_cast<Atom*>(m_selectedBond->GetEndAtom());

    // Draw bond length text.
    if (!(m_leftButtonPressed && isAtomInBond(m_clickedAtom, m_selectedBond)))
    {
      QString length = tr("Bond Length:  ") + 
                       QString::number(m_selectedBond->GetLength(), 10, 1) +
                       QString::fromUtf8(" Å (Angstrom)");

      widget->painter()->begin(widget);
      widget->painter()->drawText(QPoint(5, widget->height() - 25), length);
      widget->painter()->end();
    }

    // Draw the angles around the two atoms.
    if (!m_clickedAtom || m_midButtonPressed || (m_leftButtonPressed && begin != m_clickedAtom)
        || (m_rightButtonPressed && !isAtomInBond(m_clickedAtom, m_selectedBond)))
      drawAngles(widget, begin, m_selectedBond, mask1);

    if (!m_clickedAtom || m_midButtonPressed || (m_leftButtonPressed && end != m_clickedAtom)
         || (m_rightButtonPressed && !isAtomInBond(m_clickedAtom, m_selectedBond)))
      drawAngles(widget, end, m_selectedBond, mask1);

    // Draw the manipulation rectangle.
    Eigen::Vector3d *reference = calculateSnapTo(widget, m_selectedBond, m_referencePoint, 20);
    if (reference)
    {
      double rgb[3] = {1.0, 1.0, 0.2};
      drawManipulationRectangle(widget, m_selectedBond, reference, rgb, mask2);
      delete reference;
    }
    else
    {
      double rgb[3] = {0.0, 0.2, 0.8};
      drawManipulationRectangle(widget, m_selectedBond, m_referencePoint, rgb, mask2);
    }
  }

  return true;
}

// ##########  isAtomInBond  ##########

bool BondCentricTool::isAtomInBond(Atom *atom, Bond *bond)
{
  if (!atom || !bond)
    return false;

  if (atom == static_cast<Atom*>(bond->GetBeginAtom()))
    return true;

  return atom == static_cast<Atom*>(bond->GetEndAtom());
}

// ##########  drawAtomAngles  ##########

void BondCentricTool::drawAtomAngles(GLWidget *widget, GLubyte *mask)
{
  if (!m_clickedAtom || !widget)
    return;

  OBBondIterator bondIter = m_clickedAtom->EndBonds();

  Atom *u = (Atom*)m_clickedAtom->BeginNbrAtom(bondIter);
  Atom *v = NULL;

  if (u != NULL)
  {
    do
    {
      OBBondIterator tmpIter = bondIter;

      while ((v = (Atom*)m_clickedAtom->NextNbrAtom(tmpIter)) != NULL)
      {
        drawAngleSector(widget, m_clickedAtom->pos(), u->pos(), v->pos(), mask);
      }
    }
    while((u = (Atom*)m_clickedAtom->NextNbrAtom(bondIter)) != NULL);
  }
}

// ##########  drawAngles  ##########

void BondCentricTool::drawAngles(GLWidget *widget, Atom *atom, Bond *bond, GLubyte *mask)
{
  if (!atom || !bond)
    return;

  Atom *ref = NULL;
  if (atom == static_cast<Atom*>(bond->GetBeginAtom()))
  {
    ref = static_cast<Atom*>(bond->GetEndAtom());
  }
  else if (atom == static_cast<Atom*>(bond->GetEndAtom()))
  {
    ref = static_cast<Atom*>(bond->GetBeginAtom());
  }
  else
    return;

  OBBondIterator bondIter = atom->EndBonds();
  Atom *v = (Atom*)atom->BeginNbrAtom(bondIter);

  if (v != NULL)
  {
    do
    {
      if (v == ref)
        continue;

      drawAngleSector(widget, atom->pos(), ref->pos(), v->pos(), mask);
    }
    while ((v = (Atom*)atom->NextNbrAtom(bondIter)) != NULL);
  }
}

// ##########  drawAngleSector  ##########

void BondCentricTool::drawAngleSector(GLWidget *widget, Eigen::Vector3d origin,
                                      Eigen::Vector3d direction1, Eigen::Vector3d direction2, GLubyte *mask)
{
  widget->painter()->begin(widget);

  // Get vectors representing the lines from centre to left and centre to right.
  Eigen::Vector3d u = direction1 - origin;
  Eigen::Vector3d v = direction2 - origin;

  // Calculate the length of the vectors (half the length of the shortest vector.)
  double radius = qMin(u.norm(), v.norm()) * 0.5;
  double lineWidth = 1.5;
  if(mask != mask2)
  {
    // Adjust the length of u and v to the length calculated above.
    u = (u / u.norm()) * radius;
    v = (v / v.norm()) * radius;

    // Angle between u and v.
    double uvAngle = acos(u.dot(v) / v.norm2()) * 180.0 / M_PI;

    // If angle is less than 1 (will be approximated to 0), attempting to draw
    // will crash, so return.
    if (abs(uvAngle) <= 1)
      return;

    // Vector perpindicular to both u and v.
    Eigen::Vector3d n = u.cross(v);

    Eigen::Vector3d x = Vector3d(1, 0, 0);
    Eigen::Vector3d y = Vector3d(0, 1, 0);

    if (n.norm() < 1e-16)
    {
      Eigen::Vector3d A = u.cross(x);
      Eigen::Vector3d B = u.cross(y);

      n = A.norm() >= B.norm() ? A : B;
    }

    n = n / n.norm();

    Matrix3d rotationMatrix;
    rotationMatrix.loadRotation3((uvAngle / 2 * (M_PI / 180.0)), n);

    Vector3d point = Vector3d(0, 0, 0);
    rotationMatrix.multiply(u, &point);
  
    QString angle = QString::number(uvAngle, 10, 1) + QString::fromUtf8("°");
    widget->painter()->drawText(point + origin, angle);
    widget->painter()->end();

    widget->painter()->begin(widget);
  }
  glColor4f(0, 0.5, 0, 0.4);
  glEnable(GL_BLEND);
  glEnable(GL_POLYGON_STIPPLE);
  glPolygonStipple(mask);
  widget->painter()->drawShadedSector(origin, direction1, direction2, radius);
  glDisable(GL_POLYGON_STIPPLE);
  glDisable(GL_BLEND);

  glColor4f(1.0, 1.0, 1.0, 1.0);
  widget->painter()->drawArc(origin, direction1, direction2, radius, lineWidth);

  widget->painter()->end();
}

// ##########  calcualteSnapTo  ##########

Eigen::Vector3d* BondCentricTool::calculateSnapTo(GLWidget *widget, Bond *bond, Eigen::Vector3d *referencePoint, double maximumAngle)
{
  if(!referencePoint || !bond || !widget)
  {
    return NULL;
  }
  double angle = -1;
  Eigen::Vector3d *smallestRef = NULL;
  Atom *b = static_cast<Atom*>(bond->GetBeginAtom());
  Atom *e = static_cast<Atom*>(bond->GetEndAtom());

  OBBondIterator bondIter = b->EndBonds();
  Atom *t = (Atom*)b->BeginNbrAtom(bondIter);

  Eigen::Vector3d begin = b->pos();
  Eigen::Vector3d end = e->pos();
  Eigen::Vector3d target;
  if (t != NULL)
  {
    do
    {
      if (t == e)
        continue;
      target = t->pos();
      Eigen::Vector3d u = end - begin;
      Eigen::Vector3d v = target - begin;
      double tAngle = acos(u.dot(v) / (v.norm() * u.norm())) * 180.0 / M_PI;
      if(!(tAngle > 1 && tAngle < 179))
        continue;
      Eigen::Vector3d orth1 = u.cross(v);
      Eigen::Vector3d orth2 = referencePoint->cross(v);
      tAngle = acos(orth1.dot(orth2) / (orth1.norm() * orth2.norm())) * 180.0 / M_PI;
      tAngle = tAngle > 90 ? 180 - tAngle : tAngle;
      if(angle < 0)
      {
        angle = tAngle;
        smallestRef = new Vector3d(v);
      }
      else if(tAngle < angle)
      {
        angle = tAngle;
        delete smallestRef;
        smallestRef = new Vector3d(v);
      }
    }
    while ((t = (Atom*)b->NextNbrAtom(bondIter)) != NULL);
  }
  
  bondIter = e->EndBonds();
  t = (Atom*)e->BeginNbrAtom(bondIter);
  
  if (t != NULL)
  {
    do
    {
      if (t == b)
        continue;
      target = t->pos();
      Eigen::Vector3d u = begin - end;
      Eigen::Vector3d v = target - end;
      double tAngle = acos(u.dot(v) / (v.norm() * u.norm())) * 180.0 / M_PI;
      if(!(tAngle > 1 && tAngle < 179))
        continue;
      Eigen::Vector3d orth1 = u.cross(v);
      Eigen::Vector3d orth2 = referencePoint->cross(v);
      tAngle = acos(orth1.dot(orth2) / (orth1.norm() * orth2.norm())) * 180.0 / M_PI;
      tAngle = tAngle > 90 ? 180 - tAngle : tAngle;
      if(angle < 0)
      {
        angle = tAngle;
        smallestRef = new Vector3d(v);
      }
      else if(tAngle < angle)
      {
        angle = tAngle;
        delete smallestRef;
        smallestRef = new Vector3d(v);
      }
    }
    while ((t = (Atom*)e->NextNbrAtom(bondIter)) != NULL);
  }
  if (angle > maximumAngle)
  {
    if (smallestRef)
    {
      delete smallestRef;
    }
    return NULL;
  }
  
  return smallestRef;
}

// ##########  drawManipulationRectangle  ##########

void BondCentricTool::drawManipulationRectangle(GLWidget *widget, Bond *bond, Eigen::Vector3d *&referencePoint, double rgb[3], GLubyte *mask)
{
  if (!bond || !widget)
    return;

  Atom *leftAtom = static_cast<Atom*>(bond->GetBeginAtom());
  Atom *rightAtom = static_cast<Atom*>(bond->GetEndAtom());

  Eigen::Vector3d left = leftAtom->pos();
  Eigen::Vector3d right = rightAtom->pos();

  Eigen::Vector3d leftToRight = right - left;

  if (!referencePoint)
  {
    Eigen::Vector3d x = Vector3d(1, 0, 0);
    Eigen::Vector3d y = Vector3d(0, 1, 0);

    Eigen::Vector3d A = leftToRight.cross(x);
    Eigen::Vector3d B = leftToRight.cross(y);

    referencePoint = A.norm() >= B.norm() ? new Vector3d(A) : new Vector3d(B);
    *referencePoint = *referencePoint / referencePoint->norm();
  }
  
  Eigen::Vector3d vec = leftToRight.cross(*referencePoint);
  Eigen::Vector3d planeVec = vec.cross(leftToRight);

  double length = 1;

  planeVec = length * (planeVec / planeVec.norm());

  Eigen::Vector3d topLeft = widget->camera()->modelview() * (left + planeVec);
  Eigen::Vector3d topRight = widget->camera()->modelview() * (right + planeVec);
  Eigen::Vector3d botRight = widget->camera()->modelview() * (right - planeVec);
  Eigen::Vector3d botLeft = widget->camera()->modelview() * (left - planeVec);

  float alpha = 0.4;
  double lineWidth = 1.5;

  widget->painter()->begin(widget);

  glEnable(GL_BLEND);
  glColor4f(rgb[0], rgb[1], rgb[2], alpha);
  glEnable(GL_POLYGON_STIPPLE);
  glPolygonStipple(mask);
  widget->painter()->drawShadedQuadrilateral(topLeft, topRight, botRight, botLeft);
  glDisable(GL_POLYGON_STIPPLE);
  glDisable(GL_BLEND);
  glColor4f(1.0, 1.0, 1.0, 1.0);
  widget->painter()->drawQuadrilateral(topLeft, topRight, botRight, botLeft, lineWidth);

  widget->painter()->end();
}

// ##########  drawSphere  ##########

void BondCentricTool::drawSphere(GLWidget *widget,  const Eigen::Vector3d &position, double radius, float alpha )
{
  widget->painter()->begin(widget);
  Color( 1.0, 1.0, 0.3, alpha ).applyAsMaterials();
  glEnable(GL_BLEND);
  widget->painter()->drawSphere(position, radius);
  glDisable(GL_BLEND);
  widget->painter()->end();
}

#include "bondcentrictool.moc"

Q_EXPORT_PLUGIN2(bondcentrictool, BondCentricToolFactory)
