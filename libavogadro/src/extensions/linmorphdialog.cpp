/**********************************************************************
  LinMorphDialog - dialog for lin morph extension
  Copyright (C) 2008 by Naomi Fox

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

#include "linmorphdialog.h"

#include <QPushButton>
#include <QButtonGroup>
#include <QDebug>

#include <QFileDialog>
#include <QFile>

#include <QMessageBox>
#include <QInputDialog>

#include <openbabel/plugin.h>

using namespace OpenBabel;

namespace Avogadro {

  LinMorphDialog::LinMorphDialog( QWidget *parent, Qt::WindowFlags f ) : QDialog( parent, f )//, m_movieDialog(0)
  {
    //  qDebug() << "LinMorphDialog::LinMorphDialog()" << endl;
    ui.setupUi(this);
    connect(ui.loadButton, SIGNAL(clicked()), this, SLOT(loadFile()));
    connect(ui.frameSlider, SIGNAL(valueChanged(int)), this, SIGNAL(sliderChanged(int)));
    connect(ui.fpsSpin, SIGNAL(valueChanged(int)), this, SIGNAL(fpsChanged(int)));
    connect(ui.loopBox, SIGNAL(stateChanged(int)), this, SIGNAL(loopChanged(int)));
    connect(ui.numFramesSpin, SIGNAL(valueChanged(int)), this, SIGNAL(frameCountChanged(int)));    
    connect(ui.saveSnapshotsButton, SIGNAL(clicked()), this, SLOT(savePovSnapshots()));
    connect(ui.saveMovieButton, SIGNAL(clicked()), this, SLOT(saveMovie()));
    connect(ui.playButton, SIGNAL(clicked()), this, SIGNAL(play()));
    connect(ui.pauseButton, SIGNAL(clicked()), this, SIGNAL(pause()));
    connect(ui.stopButton, SIGNAL(clicked()), this, SIGNAL(stop()));
    
  }

  LinMorphDialog::~LinMorphDialog()
  {
    //  qDebug() << "LinMorphDialog::~LinMorphDialog()" << endl;
  }

  void LinMorphDialog::loadFile()
  {
    QString file = 
      QFileDialog::getOpenFileName(this,
				   tr("Open mol file"), ui.fileEdit->text(),
				   tr("Any files (*.*)"));
    ui.fileEdit->setText(file); 
    emit fileName(file);
  }


  void LinMorphDialog::saveMovie()
  {
    QString sMovieFileName = QFileDialog::getSaveFileName(this, 
							  tr("Save Movie File"),
							  ui.movieFileLine->text(),
							  tr("movie files (*.avi)"));

    if (!sMovieFileName.isEmpty() ) 
      ui.movieFileLine->setText(sMovieFileName);

    emit movieFileInfo(sMovieFileName);

  }

  void LinMorphDialog::savePovSnapshots()
  {
    QString ssDirectory = 
      QFileDialog::getExistingDirectory(this, 
					tr("Open Directory"),
					ui.fileEdit->text(),
					QFileDialog::ShowDirsOnly
					| QFileDialog::DontResolveSymlinks);    
    
    bool ok;
    QString ssPrefixText = 
      QInputDialog::getText(this, tr("QInputDialog::getText()"),
			    tr("prefix for pov files"), 
			    QLineEdit::Normal,
			    QDir::home().dirName(), &ok);
    
    QString ssFullPrefixText;
    if (ok && !ssPrefixText.isEmpty() && !ssDirectory.isEmpty()) {
      ssFullPrefixText = ssDirectory + ssPrefixText;
      ui.fullPrefixLine->setText(ssFullPrefixText);
    }
    else {
      QMessageBox::warning( NULL, tr( "Avogadro" ),
			    tr( "Directory and prefix for snapshots not given" ));
      return;
    }    
    emit snapshotsPrefix(ssFullPrefixText);
  }
  
  void LinMorphDialog::setFrame(int i)
  {
    QString str = tr("%1/%2").arg( i ).arg( m_frameCount );
    ui.frameEdit->setText(str);
    ui.frameSlider->setValue(i);
  }

  void LinMorphDialog::setFrameCount(int i)
  {
    m_frameCount = i;
    ui.frameSlider->setMaximum(i);
    ui.numFramesSpin->setValue(i);
    setFrame(1);
  }

  int LinMorphDialog::fps()
  {
    return ui.fpsSpin->value();
  }
}

#include "linmorphdialog.moc"
