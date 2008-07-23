/**********************************************************************
  DrawTool - Tool for drawing molecules

  Copyright (C) 2007 Donald Ephraim Curtis
  Copyright (C) 2008 Tim Vandermeersch

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

#ifndef DRAWTOOL_H
#define DRAWTOOL_H

#include <avogadro/glwidget.h>
#include <avogadro/tool.h>
#include <avogadro/periodictableview.h>
#include "insertfragmentdialog.h"

#include <QGLWidget>
#include <QObject>
#include <QStringList>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QImage>
#include <QAction>
#include <QUndoCommand>
#include <QCheckBox>

#include <openbabel/forcefield.h>

namespace Avogadro {

  class AddAtomCommand;
  class DrawTool : public Tool
  {
    Q_OBJECT

    public:
      //! Constructor
      DrawTool(QObject *parent = 0);
      //! Deconstructor
      virtual ~DrawTool();

      //! \name Description methods
      //@{
      //! Tool Name (ie DrawTool)
      virtual QString name() const { return(tr("Draw")); }
      //! Tool Description (ie. DrawTools atoms and bonds)
      virtual QString description() const { return(tr("Draws Things")); }
      //@}

      //! \name Tool Methods
      //@{
      //! \brief Callback methods for ui.actions on the canvas.

      /*! Handle a mouse press (i.e., beginning of drawing)
      */
      virtual QUndoCommand* mousePress(GLWidget *widget, const QMouseEvent *event);
      /*! Handle a mouse release (i.e., the end of drawing)
      */
      virtual QUndoCommand* mouseRelease(GLWidget *widget, const QMouseEvent *event);
      /*! Handle a mouse move (perhaps drawing a bond)
      */
      virtual QUndoCommand* mouseMove(GLWidget *widget, const QMouseEvent *event);
      /*! Handle a scroll wheel (i.e., zooming in and out of the canvas)
      */
      virtual QUndoCommand* wheel(GLWidget *widget, const QWheelEvent *event);
      //@}

      /**
       * Write the tool settings so that they can be saved between sessions.
       */
      virtual void writeSettings(QSettings &settings) const;

      /**
       * Read in the settings that have been saved for the tool instance.
       */
      virtual void readSettings(QSettings &settings);

      //! The priority of the tool in the toolbar
      virtual int usefulness() const;

      virtual QWidget *settingsWidget();

      void setElement(int i);
      int element() const;

      int bondOrder() const;
      int addHydrogens() const;

    public Q_SLOTS:
      void setAddHydrogens( int state );

      void elementChanged( int index );
      void customElementChanged( int index );
      void bondOrderChanged( int index );
      void setBondOrder(int i);
      void setInsertFragmentMode( bool mode );

    private:
      Qt::MouseButtons    _buttons;

      bool                m_movedSinceButtonPressed;

      QPoint              m_initialDraggingPosition;
      QPoint              m_lastDraggingPosition;

      bool m_beginAtomAdded;
      bool m_endAtomAdded;
      Atom *m_beginAtom;
      Atom *m_endAtom;
      int m_element;

      Bond *m_bond;
      int m_bondOrder;

      int m_prevAtomElement;

      Bond *m_prevBond;
      int m_prevBondOrder;

      int m_addHydrogens;

      bool m_insertFragmentMode;

      QList<GLHit> m_hits;

      QComboBox *m_comboElements;
      QList<int> m_elementsIndex;
      QComboBox *m_comboBondOrder;
      QCheckBox *m_addHydrogensCheck;
      QPushButton *m_tableButton;
      PeriodicTableView *m_periodicTable;
      QPushButton *m_fragmentButton;
      InsertFragmentDialog *m_fragmentDialog;
      QVBoxLayout *m_layout;

      QWidget *m_settingsWidget;

      Atom *newAtom(GLWidget *widget, const QPoint& p);
      void moveAtom(GLWidget *widget, Atom *atom, const QPoint& p);
      Bond *newBond(Molecule *molecule, Atom *beginAtom, Atom *endAtom);

      OpenBabel::OBForceField *m_forceField;

    private Q_SLOTS:
      void settingsWidgetDestroyed();
      void showFragmentDialog(bool checked);
  };

  class DrawToolFactory : public QObject, public PluginFactory
  {
    Q_OBJECT
    Q_INTERFACES(Avogadro::PluginFactory)
    AVOGADRO_TOOL_FACTORY(DrawTool, tr("Draw Tool"), tr("Draw molecules, insert smiles or fragments."))
  };

} // end namespace Avogadro

#endif
