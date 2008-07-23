/**********************************************************************
  propextension.h - Properties Plugin for Avogadro

  Copyright (C) 2007 by Tim Vandermeersch

  This file is part of the Avogadro molecular editor project.
  For more information, see <http://avogadro.sourceforge.net/>

  Some code is based on Open Babel
  For more information, see <http://openbabel.sourceforge.net/>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation version 2 of the License.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 ***********************************************************************/

#include "propextension.h"
#include <avogadro/primitive.h>
#include <avogadro/color.h>
#include <avogadro/glwidget.h>

#include <QAbstractTableModel>
#include <QHeaderView>
#include <QAction>
#include <QDialog>
#include <QVBoxLayout>

using namespace std;
using namespace OpenBabel;

namespace Avogadro
{
  enum PropExtensionIndex
    {
      AtomPropIndex = 0,
      BondPropIndex,
      AnglePropIndex,
      TorsionPropIndex,
      CartesianIndex,
      ConformerIndex
    };

  PropertiesExtension::PropertiesExtension( QObject *parent ) : Extension( parent )
  {
    QAction *action;

    action = new QAction( this );
    action->setSeparator(true);
    action->setData(-1);
    m_actions.append(action);
    
    action = new QAction( this );
    action->setText( tr("Atom Properties..." ));
    action->setData(AtomPropIndex);
    m_actions.append( action );

    action = new QAction( this );
    action->setText( tr("Bond Properties..." ));
    action->setData(BondPropIndex);
    m_actions.append( action );
    
    action = new QAction( this );
    action->setText( tr("Angles Properties..." ));
    action->setData(AnglePropIndex);
    m_actions.append( action );
    
    action = new QAction( this );
    action->setText( tr("Torsion Properties..." ));
    action->setData(TorsionPropIndex);
    m_actions.append( action );
    
    action = new QAction( this );
    action->setText( tr("Conformers..." ));
    action->setData(ConformerIndex);
    m_actions.append( action );

    action = new QAction( this );
    action->setText( tr("Cartesian Editor..." ));
    action->setData(CartesianIndex);
    m_actions.append( action );
    
    action = new QAction( this );
    action->setSeparator(true);
    action->setData(-1);
    m_actions.append(action);
  }

  PropertiesExtension::~PropertiesExtension()
  {}

  QList<QAction *> PropertiesExtension::actions() const
  {
    return m_actions;
  }

  QString PropertiesExtension::menuPath(QAction *action) const
  {
    int i = action->data().toInt();
    switch(i) {
    case AtomPropIndex:
    case BondPropIndex:
    case AnglePropIndex:
    case TorsionPropIndex:
    case ConformerIndex:
      return tr("&Build") + '>' + tr("&Properties");
    case CartesianIndex:
    default:
      return tr("&Build");
      break;
    };
    return QString();
  }

  void PropertiesExtension::setMolecule(Molecule *molecule)
  {
    m_molecule = molecule;
  }

  QUndoCommand* PropertiesExtension::performAction( QAction *action, GLWidget *widget)
  {
    QUndoCommand *undo = 0;
    PropertiesModel *model;
    PropertiesView  *view;

    int i = action->data().toInt();
    switch ( i ) {
    case AtomPropIndex: // atom properties     
      // model will be deleted in PropertiesView::hideEvent using deleteLater().
      model = new PropertiesModel(PropertiesModel::AtomType);
      model->setMolecule( m_molecule );
      // view will delete itself in PropertiesView::hideEvent using deleteLater().
      view = new PropertiesView(PropertiesView::AtomType);
      connect(m_molecule, SIGNAL( updated() ), model, SLOT( updateTable() ));
      connect(m_molecule, SIGNAL( primitiveAdded(Primitive *) ), model, SLOT( primitiveAdded(Primitive *) ));
      connect(m_molecule, SIGNAL( primitiveRemoved(Primitive *) ), model, SLOT( primitiveRemoved(Primitive *) ));
      view->setMolecule( m_molecule );
      view->setWidget( widget );
      view->setModel( model );
      view->resize(860, 400);
      view->show();
      break;
    case BondPropIndex: // bond properties
      // model will be deleted in PropertiesView::hideEvent using deleteLater().
      model = new PropertiesModel(PropertiesModel::BondType);
      model->setMolecule( m_molecule );
      // view will delete itself in PropertiesView::hideEvent using deleteLater().
      view = new PropertiesView(PropertiesView::BondType);
      connect(m_molecule, SIGNAL( updated() ), model, SLOT( updateTable() ));
      connect(m_molecule, SIGNAL( primitiveAdded(Primitive *) ), model, SLOT( primitiveAdded(Primitive *) ));
      connect(m_molecule, SIGNAL( primitiveRemoved(Primitive *) ), model, SLOT( primitiveRemoved(Primitive *) ));
      view->setMolecule( m_molecule );
      view->setWidget( widget );
      view->setModel( model );
      view->resize(550, 400);
      view->show();
      break;
    case AnglePropIndex: // angle properties
      // model will be deleted in PropertiesView::hideEvent using deleteLater().
      model = new PropertiesModel(PropertiesModel::AngleType);
      model->setMolecule( m_molecule );
      // view will delete itself in PropertiesView::hideEvent using deleteLater().
      view = new PropertiesView(PropertiesView::AngleType);
      connect(m_molecule, SIGNAL( updated() ), model, SLOT( updateTable() ));
      //connect(m_molecule, SIGNAL( primitiveAdded(Primitive *) ), model, SLOT( primitiveAdded(Primitive *) ));
      //connect(m_molecule, SIGNAL( primitiveRemoved(Primitive *) ), model, SLOT( primitiveRemoved(Primitive *) ));
      view->setMolecule( m_molecule );
      view->setWidget( widget );
      view->setModel( model );
      view->resize(550, 400);
      view->show();
      break;
    case TorsionPropIndex: // torsion properties
      // model will be deleted in PropertiesView::hideEvent using deleteLater().
      model = new PropertiesModel(PropertiesModel::TorsionType);
      model->setMolecule( m_molecule );
      // view will delete itself in PropertiesView::hideEvent using deleteLater().
      view = new PropertiesView(PropertiesView::TorsionType);
      connect(m_molecule, SIGNAL( updated() ), model, SLOT( updateTable() ));
      //connect(m_molecule, SIGNAL( primitiveAdded(Primitive *) ), model, SLOT( primitiveAdded(Primitive *) ));
      //connect(m_molecule, SIGNAL( primitiveRemoved(Primitive *) ), model, SLOT( primitiveRemoved(Primitive *) ));
      view->setMolecule( m_molecule );
      view->setWidget( widget );
      view->setModel( model );
      view->resize(550, 400);
      view->show();
      break;
    case CartesianIndex: // cartesian editor
      // m_angleModel will be deleted in PropertiesView::hideEvent using deleteLater().
      model = new PropertiesModel(PropertiesModel::CartesianType);
      model->setMolecule( m_molecule );
      // m_view will delete itself in PropertiesView::hideEvent using deleteLater().
      view = new PropertiesView(PropertiesView::CartesianType);
      connect(m_molecule, SIGNAL( updated() ), model, SLOT( updateTable() ));
      connect(m_molecule, SIGNAL( primitiveAdded(Primitive *) ), model, SLOT( primitiveAdded(Primitive *) ));
      connect(m_molecule, SIGNAL( primitiveRemoved(Primitive *) ), model, SLOT( primitiveRemoved(Primitive *) ));
      view->setMolecule( m_molecule );
      view->setWidget( widget );
      view->setModel( model );
      view->resize(360, 400);
      view->show();
      break;
    case ConformerIndex: // conformers
      QDialog *dialog = new QDialog();
      QVBoxLayout *layout = new QVBoxLayout(dialog);
      layout->setSpacing(0);
      layout->setContentsMargins(0,0,0,0);
      // model will be deleted in PropertiesView::hideEvent using deleteLater().
      model = new PropertiesModel(PropertiesModel::ConformerType, dialog);
      model->setMolecule( m_molecule );
      // view will delete itself in PropertiesView::hideEvent using deleteLater().
      view = new PropertiesView(PropertiesView::ConformerType, dialog);
      connect(m_molecule, SIGNAL( updated() ), model, SLOT( updateTable() ));
      view->setMolecule( m_molecule );
      view->setWidget( widget );
      view->setModel( model );
      view->resize(180, 500);
      view->sortByColumn(0, Qt::AscendingOrder);
      layout->addWidget(view);
      dialog->show();
      // delete the dialog when we're done.
      connect(dialog, SIGNAL(rejected()), dialog, SLOT(deleteLater()));
      break;
    }

    return undo;
  }

  PropertiesView::PropertiesView(Type type, QWidget *parent) : QTableView(parent), m_molecule(NULL), m_widget(NULL)
  {
    m_type = type;
    
    QString title;
    switch(type){
      case AtomType:
        title = tr("Atom") + ' ';
        break;
      case BondType:
        title = tr("Bond") + ' ';
        break;
      case AngleType:
        title = tr("Angle") + ' ';
        break;
      case TorsionType:
        title = tr("Torsion") + ' ';
        break;
      case CartesianType:
        title = tr("Cartesian") + ' ';
        break;
      case ConformerType:
        title = tr("Conformer") + ' ';
        break;
      default:
        title = QString();
    }
    title += tr("Properties");
    this->setWindowTitle(title);
    
    QHeaderView *horizontal = this->horizontalHeader();
    horizontal->setResizeMode(QHeaderView::Stretch);
    QHeaderView *vertical = this->verticalHeader();
    vertical->setResizeMode(QHeaderView::Stretch);
  }
  
  
  void PropertiesView::selectionChanged(const QItemSelection &selected, const QItemSelection &)
  {
    QModelIndex index;
    QList<Primitive *> matchedPrimitives;
    QModelIndexList items = selected.indexes();
    
    foreach (index, items) {
      if (!index.isValid())
        return;
    
      if (m_type == AtomType) {
        if ((unsigned int) index.row() >= m_molecule->NumAtoms())
          return;
    
        matchedPrimitives.append(static_cast<Atom*>(m_molecule->GetAtom(index.row()+1)));
        m_widget->clearSelected();
        m_widget->setSelected(matchedPrimitives, true);
        m_widget->update();
      } else if (m_type == BondType) {
        if((unsigned int) index.row() >= m_molecule->NumBonds())
          return;
        
        matchedPrimitives.append(static_cast<Bond*>(m_molecule->GetBond(index.row())));
        m_widget->clearSelected();
        m_widget->setSelected(matchedPrimitives, true);
        m_widget->update();
      } else if (m_type == ConformerType) {
        if (index.row() >= m_molecule->NumConformers())
          return;
    
        m_molecule->SetConformer(index.row());
        m_molecule->update();
        return; 
      } 
    }
  }
  
  void PropertiesView::setMolecule(Molecule *molecule)
  {
    m_molecule = molecule;
  }
  
  void PropertiesView::setWidget(GLWidget *widget)
  {
    m_widget = widget;
  }
  
  void PropertiesView::hideEvent(QHideEvent *)
  {
    if (m_widget)
      m_widget->clearSelected();
    
    QAbstractItemModel *m_model = model();
    if (m_model)
      m_model->deleteLater();

    this->deleteLater();
  }
  
} // end namespace Avogadro

#include "propextension.moc"
Q_EXPORT_PLUGIN2( propextension, Avogadro::PropertiesExtensionFactory )
