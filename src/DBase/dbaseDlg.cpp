//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: dbaseDlg.cpp,v 1.31 2010/09/01 23:54:19 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %DBaseDlg class
 */
#include "dbaseDlg.h"

#include <algorithm>

#include <utility>
#include <QFileDialog>
#include <QDir>
#include <QComboBox>

#include "graspitGUI.h"
#include "ivmgr.h"
#include "robot.h"
#include "world.h"
#include "searchState.h"
#include "grasp.h"
#include "graspitGUI.h"
#include "mainWindow.h"
#include "matvec3D.h"


#include "DBPlanner/sql_database_manager.h"
#ifdef ROS_DATABASE_MANAGER
#include "DBPlanner/ros_database_manager.h"
#endif

#include "graspit_db_model.h"
#include "graspit_db_grasp.h"
#include "dbasePlannerDlg.h"
#include "graspCaptureDlg.h"

//#define GRASPITDBG
#include "debug.h"

//#define PROF_ENABLED
#include "profiling.h"

/*! Initializes the dialog and also gets the one and only manager from the
	GraspitGUI. If this manager is already set, it also loads the model 
	list from the database and initializes it.
*/
void DBaseDlg::init()
{
	mModelList.clear();
	mGraspList.clear();
	mGraspList.clear();
	browserGroup->setEnabled(FALSE);
	graspsGroup->setEnabled(FALSE);
	mDBMgr = graspItGUI->getIVmgr()->getDBMgr();
	if (mDBMgr) {
		getModelList();
	}
	sortBox->insertItem("Epsilon");
	sortBox->insertItem("Volume");
	sortBox->insertItem("Energy");
}

void DBaseDlg::destroy()
{
	//we do not delete the dbmgr because it is now set in the ivmgr for the rest of
	//graspit to use. The ivmgr deletes it on its exit.
	delete mModelScene;
}

void DBaseDlg::exitButton_clicked(){
	if (mCurrentLoadedModel) {
		//remove the previously loaded model, but don't delete it
		graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentLoadedModel->getGraspableBody(), false);
	}
	//delete and release all the memories occupied by the grasps
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
	mGraspList.clear();
	//delete and release all the memories occupied by the models
	deleteVectorElements<db_planner::Model*, GraspitDBModel*>(mModelList);
	QDialog::accept();
}

bool compareNames(db_planner::Model *m1, db_planner::Model *m2)
{
  return m1->ModelName() < m2->ModelName();
}

void DBaseDlg::getModelList()
{
	//clear the modelList
	deleteVectorElements<db_planner::Model*, GraspitDBModel*>(mModelList);
	mModelList.clear();
	//load the models from database manager
	if(!mDBMgr->ModelList(&mModelList,db_planner::FilterList::NONE)){
		DBGA("Model list retrieval failed");
		return;
	}
        //sort by names
        std::sort(mModelList.begin(), mModelList.end(), compareNames);
	//display the retrieved models, put their names into the combobox
	displayModelList();
	//check that there are valid number of models
	if(!mModelList.empty()){
		browserGroup->setEnabled(TRUE);
		connectButton->setEnabled(FALSE);
	}
	std::vector<std::string>graspTypes;
	if(!mDBMgr->GraspTypeList(&graspTypes)){
		DBGA("Grasp Types not loaded");
		return;
	}
	//display the types
	displayGraspTypeList(graspTypes);
}

/*! Deletes the old connection to the database and creates a new one based
	on the current settings in the dialog box. The new connection is then
	set as the one an only Database Manager that the rest of GraspIt had
	acces to.

	After connecting, it also reads the model list form the database and
	displays it.
*/
void DBaseDlg::connectButton_clicked()
{
	delete mDBMgr;
	Hand *h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();

#ifndef ROS_DATABASE_MANAGER	
	mDBMgr = new db_planner::SqlDatabaseManager(hostLineEdit->text().toStdString(),
						    atoi(portLineEdit->text().latin1()),
						    usernameLineEdit->text().toStdString(),
						    passwordLineEdit->text().toStdString(),
						    databaseLineEdit->text().toStdString(),
						    new GraspitDBModelAllocator(),
						    new GraspitDBGraspAllocator(h));
#else
	mDBMgr = new db_planner::RosDatabaseManager(hostLineEdit->text().toStdString(),
						    portLineEdit->text().toStdString(),
						    usernameLineEdit->text().toStdString(),
						    passwordLineEdit->text().toStdString(),
						    databaseLineEdit->text().toStdString(),
						    new GraspitDBModelAllocator(),
						    new GraspitDBGraspAllocator(h));
        //use the special allocator for models that get geometry directly from the database
        GeomGraspitDBModelAllocator* allocator = new GeomGraspitDBModelAllocator(mDBMgr);
        mDBMgr->SetModelAllocator(allocator);
#endif

	if (mDBMgr->isConnected()) {
		getModelList();
	} else {
		DBGA("DBase Browser: Connection failed");
		delete mDBMgr;
		mDBMgr = NULL;
	}
	graspItGUI->getIVmgr()->setDBMgr(mDBMgr);
}

PROF_DECLARE(GET_GRASPS);
PROF_DECLARE(GET_GRASPS_CALL);

void DBaseDlg::loadGraspButton_clicked(){
	PROF_RESET_ALL;
	PROF_START_TIMER(GET_GRASPS);
	//get the current hand and check its validity
	Hand *hand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	if (!hand) {
		DBGA("Load and select a hand before viewing grasps!");
		return;
	}
	//check the currently loaded model
	if(!mCurrentLoadedModel){
		DBGA("Load model first!");
		return;
	}
	//clear the previously loaded grasps
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
	mGraspList.clear();
	mCurrentFrame = 0;
	//get new grasps from database manager
	PROF_START_TIMER(GET_GRASPS_CALL);
	if(!mDBMgr->GetGrasps(*mCurrentLoadedModel,hand->getDBName().toStdString(), &mGraspList)){
		DBGA("Load grasps failed");
		mGraspList.clear();
		return;
	}
	PROF_STOP_TIMER(GET_GRASPS_CALL);
	for(std::vector<db_planner::Grasp*>::iterator it = mGraspList.begin(); it != mGraspList.end(); ){
		if( QString((*it)->GetSource().c_str()) == typesComboBox->currentText() ||
			typesComboBox->currentText() == "ALL") ++it;
		else{
			delete (*it);
			mGraspList.erase(it);
		}
	}
	//set corresponding indices and show the grasp
	QString numTotal, numCurrent;
	numTotal.setNum(mGraspList.size());
	if(!mGraspList.empty()){
		numCurrent.setNum(mCurrentFrame + 1);
		graspsGroup->setEnabled(TRUE);
		showGrasp(0);
		showMarkers();
	} else{
		numCurrent.setNum(0);
		graspsGroup->setEnabled(FALSE);
	}
	graspIndexLabel->setText(numCurrent + "/" + numTotal);
	PROF_STOP_TIMER(GET_GRASPS);
	PROF_PRINT_ALL;
}

void DBaseDlg::loadModelButton_clicked(){
	if (mCurrentLoadedModel) {
		//remove the previously loaded model, but don't delete it
		graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentLoadedModel->getGraspableBody(), false);
		mCurrentLoadedModel = NULL;
	}
	if(mModelList.empty()){
		DBGA("No model loaded...");
		return;
	}

	//check out the model in the modelList
	GraspitDBModel* model = dynamic_cast<GraspitDBModel*>(mModelList[mModelMap[modelsComboBox->currentText().toStdString()]]);
	if(!model){
		DBGA("Cannot recognize the model type");
		return;
	}
	//check that this model is already loaded into Graspit, if not, load it
	if (!model->geometryLoaded()) {
		//this loads the actual geometry in the scene graph of the object
		if ( model->load(graspItGUI->getIVmgr()->getWorld()) != SUCCESS) {
			DBGA("Model load failed");
			return;
		}
	}
	//adds the object to the collision detection system
	model->getGraspableBody()->addToIvc();
	//todo: where to dynamic information come from?
	//model->getGraspableBody()->initDynamics();
	//this adds the object to the graspit world so that we can see it
	graspItGUI->getIVmgr()->getWorld()->addBody(model->getGraspableBody());
	//and remember it
	mCurrentLoadedModel = model;
	//model->getGraspableBody()->showAxes(false);
	model->getGraspableBody()->setTransparency(0.0);
        model->getGraspableBody()->setTran(transf::IDENTITY);
	graspsGroup->setEnabled(FALSE);

	//delete the previously loaded grasps
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
	mGraspList.clear();	
	//initialize the grasp information for the new model
	initializeGraspInfo();
}

// go to see the next grasp
void DBaseDlg::nextGraspButton_clicked(){
	nextGrasp();
}

// go back to the previous grasp
void DBaseDlg::previousGraspButton_clicked(){
	previousGrasp();
}

// pop up the new window for the grasp planner
void DBaseDlg::plannerButton_clicked(){
	//check the existance of database manager
	if(!mDBMgr){
		DBGA("No dbase manager.");
		return;
	}
	//check the hand
	Hand *h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	if(!h){
		DBGA("No hand found currently");
		return;
	}
	//check the current model
	if(!mCurrentLoadedModel){
		DBGA("No object loaded");
		return;
	}
	//instantialize a new dialogue of type DBasePlannerDlg and pop it up
	DBasePlannerDlg *dlg = new DBasePlannerDlg(this, mDBMgr, mCurrentLoadedModel, h);
	dlg->setAttribute(Qt::WA_ShowModal, false);
	dlg->setAttribute(Qt::WA_DeleteOnClose, true);
	dlg->show();

	//delete the grasps loaded, release the memories, and reset the grasp information
	graspsGroup->setEnabled(FALSE);
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
	initializeGraspInfo();
}

class VisualQualityFunctor {
public:
	enum Type{ENERGY, EPSILON, VOLUME};
	Type mType;
	double operator()(const db_planner::Grasp *grasp) {
		switch(mType) {
		case ENERGY:
			return grasp->Energy();
		case EPSILON:
			return grasp->EpsilonQuality();
		case VOLUME:
			return grasp->VolumeQuality();
		default:
			return 0.0;
		}
	}
};

void DBaseDlg::sortButton_clicked()
{
	if (mGraspList.empty()) return;
	VisualQualityFunctor func;
	if (sortBox->currentText()== "Energy") {
		std::sort(mGraspList.begin(), mGraspList.end(), db_planner::Grasp::CompareEnergy);
		func.mType = VisualQualityFunctor::ENERGY;
	} else if (sortBox->currentText()== "Epsilon") {
		std::sort(mGraspList.begin(), mGraspList.end(), db_planner::Grasp::CompareEpsilon);
		func.mType = VisualQualityFunctor::EPSILON;
	} else if (sortBox->currentText()== "Volume") {
		std::sort(mGraspList.begin(), mGraspList.end(), db_planner::Grasp::CompareVolume);
		func.mType = VisualQualityFunctor::VOLUME;
	} else {
		assert(0);
	}
	mCurrentFrame = 0;
	showGrasp(mCurrentFrame);
	if (showMarkersBox->isChecked()) {
		double min = func(mGraspList.front());
		double max = func(mGraspList.back());
		if (max==min) max=min+1.0;
		std::vector<db_planner::Grasp*>::iterator it;
		for (it=mGraspList.begin(); it!=mGraspList.end(); it++) {
			double r = (func(*it) - min) / (max - min);
			double g = 1-r;
			double b = 0.0;
			GraspPlanningState *state;
			if(showPreGraspRadioButton->isChecked()) {
				state = static_cast<GraspitDBGrasp*>(*it)->getPreGraspPlanningState();
			} else {
				state = static_cast<GraspitDBGrasp*>(*it)->getFinalGraspPlanningState();
			}
			state->setIVMarkerColor(r,g,b);
		}
	}
}

//a shortcut for the GWS display
void DBaseDlg::createGWSButton_clicked(){
	graspItGUI->getMainWindow()->graspCreateProjection();
}

//trigger when the selection in the model list combo box is changed, display the corresponding new image
void DBaseDlg::modelChanged(){
	if(inModelConstruction) return;
	QString psbModelThumbPath = QString( mModelList[mModelMap[modelsComboBox->currentText().toStdString()]]->
					     ThumbnailPath().c_str() );
	if(mModelScene) delete mModelScene;
	mModelScene = new QGraphicsScene;
	mModelScene->setBackgroundBrush(Qt::blue);
	QPixmap lPixmap;
	lPixmap.load(psbModelThumbPath);
	//resize so that it will fit in window
	if (lPixmap.width() > 160) {
		lPixmap = lPixmap.scaledToWidth(160);
	}
	if (lPixmap.height() > 120) {
		lPixmap = lPixmap.scaledToHeight(120);
	}
	mModelScene->addPixmap(lPixmap);
	this->objectGraph->setScene(mModelScene);
	this->objectGraph->show();
}

void DBaseDlg::showMarkers()
{
	std::vector<db_planner::Grasp*>::iterator it;
	for (it=mGraspList.begin(); it!=mGraspList.end(); it++) {
		GraspitDBGrasp *grasp = static_cast<GraspitDBGrasp*>(*it);
		if (showPreGraspRadioButton->isChecked()) {
			grasp->getFinalGraspPlanningState()->hideVisualMarker();
			if (showMarkersBox->isChecked()) {
				grasp->getPreGraspPlanningState()->showVisualMarker();
				//show markers in blue
				if (grasp->ClusterRep()) {
				  grasp->getPreGraspPlanningState()->setIVMarkerColor(0,0,1);
				} else if (grasp->CompliantCopy()) {
				  grasp->getPreGraspPlanningState()->setIVMarkerColor(1.0,1.0,0.0);
                                } else {
				  grasp->getPreGraspPlanningState()->setIVMarkerColor(0.5,0.5,0.5);
				}				
			} else {
				grasp->getPreGraspPlanningState()->hideVisualMarker();
			}
		} else {
			grasp->getPreGraspPlanningState()->hideVisualMarker();
			if (showMarkersBox->isChecked()) {
				grasp->getFinalGraspPlanningState()->showVisualMarker();
				//show markers in blue
				if (grasp->ClusterRep()) {
				  grasp->getFinalGraspPlanningState()->setIVMarkerColor(0,0,1);
				} else if (grasp->CompliantCopy()) {
				  grasp->getFinalGraspPlanningState()->setIVMarkerColor(1.0,1.0,0.0);
				} else {
				  grasp->getFinalGraspPlanningState()->setIVMarkerColor(0.5,0.5,0.5);
				}				
			} else {
				grasp->getFinalGraspPlanningState()->hideVisualMarker();
			}
		}
	}
}

//trigger when the grasp type is changed between pregrasp and final grasp 
void DBaseDlg::graspTypeChanged(){
	showGrasp(mCurrentFrame);
	showMarkers();
}

//trigger when the model class is changed, reconstruct the model list combo box
void DBaseDlg::classChanged(){
	inModelConstruction = true;
	modelsComboBox->clear();
	for(size_t i = 0; i < mModelList.size(); ++i){
		if(mModelList[i]->Tags().find(classesComboBox->currentText().toStdString()) != mModelList[i]->Tags().end()
			|| classesComboBox->currentText() == "ALL")
			modelsComboBox->addItem(mModelList[i]->ModelName().c_str());
	}
	inModelConstruction = false;
	modelChanged();
}

//synthesize the model list combo box
void DBaseDlg::displayModelList(){
	std::set<string> tags;
	mModelMap.clear();
	for(int i = 0; i < (int)mModelList.size(); ++i){
		modelsComboBox->insertItem(QString(mModelList[i]->ModelName().c_str()));
		tags.insert(mModelList[i]->Tags().begin(), mModelList[i]->Tags().end());
		mModelMap.insert(std::make_pair<std::string, int>(mModelList[i]->ModelName(), i));
	}
	classesComboBox->clear();
	classesComboBox->insertItem("ALL");
	for(std::set<string>::iterator i = tags.begin(); i != tags.end(); ++i){
		classesComboBox->insertItem(QString((*i).c_str()));
	}
}

void DBaseDlg::displayGraspTypeList(std::vector<std::string> list){
	typesComboBox->clear();
	typesComboBox->insertItem("ALL");
	for(size_t i = 0; i < list.size(); ++i){
		typesComboBox->insertItem(QString(list[i].c_str()));
	}
}

//core routine that shows the i-th loaded grasp
void DBaseDlg::showGrasp(int i)
{
	if (mGraspList.empty()) return;
	assert( i>=0 && i < (int)mGraspList.size() );
	//put the model in correct place
	mCurrentLoadedModel->getGraspableBody()->setTran(transf::IDENTITY);
	//show the pregrasp or final grasp
	if(showPreGraspRadioButton->isChecked()){
		if(!static_cast<GraspitDBGrasp*>(mGraspList[i])->getPreGraspPlanningState())//NULL grasp, return
			return;
		static_cast<GraspitDBGrasp*>(mGraspList[i])->getPreGraspPlanningState()->execute();
	}
	else{
		if(!static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState())//NULL grasp, return
			return;
		static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState()->execute();
		if(graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->isA("Barrett")){
			graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->autoGrasp(true);
		}
	}

	/*
	std::cerr << "Original tran: " << static_cast<GraspitDBGrasp*>(mGraspList[i])			
	  ->getFinalGraspPlanningState()->getPosition()->getCoreTran() << "\n";
	GraspPlanningState *state = new GraspPlanningState(static_cast<GraspitDBGrasp*>(mGraspList[i])			
							   ->getFinalGraspPlanningState() );
	state->setPositionType(SPACE_AXIS_ANGLE, true);
	std::cerr << "     New tran: " << state->getPosition()->getCoreTran() << "\n";
	*/

	//update the world and grasp information
	graspItGUI->getIVmgr()->getWorld()->findAllContacts();
	graspItGUI->getIVmgr()->getWorld()->updateGrasps();
	mCurrentFrame = i;
	updateGraspInfo();
}

//go to see the next grasp and show the corresponding image
void DBaseDlg::nextGrasp() {
	if (mGraspList.empty()) return;
	mCurrentFrame ++;
	if (mCurrentFrame == mGraspList.size()) mCurrentFrame = 0;
	showGrasp(mCurrentFrame);
}

//go to see the previous grasp and show the corresponding image
void DBaseDlg::previousGrasp() {
	if (mGraspList.empty()) return;
	mCurrentFrame --;
	if (mCurrentFrame < 0) mCurrentFrame = mGraspList.size() - 1;
	showGrasp(mCurrentFrame);
}

//update the information of current grasp, including indices, epsilon qualities, and volume qualities
void DBaseDlg::updateGraspInfo(){
	QString numTotal, numCurrent;
	numTotal.setNum(mGraspList.size());
	if(!mGraspList.empty())
		numCurrent.setNum(mCurrentFrame + 1);
	else
		numCurrent.setNum(0);
	graspIndexLabel->setText(numCurrent + "/" + numTotal);

	QString eq, vq, en, cl;
	eq.setNum(mGraspList[mCurrentFrame]->EpsilonQuality());
	vq.setNum(mGraspList[mCurrentFrame]->VolumeQuality());
	en.setNum(mGraspList[mCurrentFrame]->Energy());
	cl.setNum(mGraspList[mCurrentFrame]->Clearance());
	
	epsilonQualityLabel->setText(QString("Epsilon Quality: " + eq));
	volumeQualityLabel->setText(QString("Volume Quality: " + vq));
	energyLabel->setText(QString("Energy: " + en));
	clearanceLabel->setText(QString("Pregrasp clearance: " + cl));
}

//reset the grasp information displayed
void DBaseDlg::initializeGraspInfo(){
	graspIndexLabel->setText("0/0");
	epsilonQualityLabel->setText(QString("Epsilon Quality: 0.0"));
	volumeQualityLabel->setText(QString("Volume Quality: 0.0"));
}

//helper function that deletes the vector of type vectorType, but treating every elements as type treatAsType
template <class vectorType, class treatAsType>
inline void DBaseDlg::deleteVectorElements(std::vector<vectorType>& v){
	for(size_t i = 0; i < v.size(); ++i){
		delete (treatAsType)v[i];
	}
	v.clear();
}

/*! Saves the current hand configuration in a binvox with dimensions
matching the currently loaded graspable body.
Note: This function can only be run when there is one robot (a hand)
in the scene and only on graspable body that also must have a binvox
member variable loaded.
*/
void
DBaseDlg::saveBinvoxButton_clicked() {
    QString fileName;
    QString fn = QFileDialog::getSaveFileName(this, QString(), QString(getenv("GRASPIT")),
                                              "GraspIt World Files (*.binvox)" );
    if ( !fn.isEmpty() ) {
      fileName = fn;
      if (fileName.section('.',1).isEmpty()) {
        fileName.append(".binvox");
      }

    } else {
        DBGA("Error saving binvox... you didn't name it properly...");
        return;
    }


    World *w = graspItGUI->getIVmgr()->getWorld();

    if (w->getNumHands() != 1) {
        DBGA("saveHandVox: There can only be one hand loaded in the world when exporting a binvox.");
        return;
    } else if (w->getNumGB() != 1) {
        DBGA("saveHandVox: There can only be one graspable body loaded in the world when exporting a binvox.");
        return;
    } else if (!mCurrentLoadedModel->binvoxLoaded()) {
        DBGA("saveHandVox: Body does not have a .binvox loaded.");
        return;
    } else if (mCurrentLoadedModel->getBinvox()->version != 1) {
        DBGA("saveHandVox: Loaded .binvox is not version 1. We only support version 1 of binvox.");
        return;
    }

    Binvox *b = mCurrentLoadedModel->getBinvox();

    DBGA("Attempting to export .binvox");


    std::ofstream binvoxOutput(fileName.toStdString().c_str(), std::ios::out | std::ios::app | std::ios::binary);

    if (binvoxOutput.is_open())
    {
        binvoxOutput << "#binvox 1\n";

        binvoxOutput << "dim " << b->depth << " " << b->height << " " << b->width << "\n";
        binvoxOutput << "translate " << b->tx << " " << b->ty << " " << b->tz << "\n";
        binvoxOutput << "scale " << b->scale << "\n";
        binvoxOutput << "data\n";

        // Done!
        DBGA("saveHandVox: Done writing .binvox")
        binvoxOutput.close();

    } else {
        QTWARNING("could not open " + fileName + "for writing");
    }


    /*

    int i,j,k,l;

    if (!file.open(QIODevice::WriteOnly)) {
        QTWARNING("could not open " + filename + "for writing");
        return FAILURE;
    }
    QTextStream stream( &file );
    stream << "<?xml version=\"1.0\" ?>" <<endl;
    stream << "<world>" << endl;
    for (i=0;i<numBodies;i++) {
        if (bodyVec[i]->isA("Body")) {
            stream<<"\t<obstacle>"<<endl;
            if(bodyVec[i]->getFilename()=="unspecified"){
                stream<<"\t\t<body>"<<endl;
                if(bodyVec[i]->saveToXml(stream)==FAILURE){
                    QTWARNING("Failed to save body info");
                    return FAILURE;
                }
                stream<<"\t\t</body>"<<endl;
            }
            else
                stream<<"\t\t<filename>"<<bodyVec[i]->getFilename().latin1()<<"</filename>"<<endl;
            stream<<"\t\t<transform>" <<endl;
            stream<< "\t\t\t<fullTransform>"<< bodyVec[i]->getTran() << "</fullTransform>" << endl;
            stream<<"\t\t</transform>" <<endl;
            stream<<"\t</obstacle>"<<endl;
        }
        else if (bodyVec[i]->inherits("GraspableBody")) {
            stream<<"\t<graspableBody>"<<endl;
            if(bodyVec[i]->getFilename()=="unspecified"){
                stream<<"\t\t<body>"<<endl;
                if(bodyVec[i]->saveToXml(stream)==FAILURE){
                    QTWARNING("Failed to save body info");
                    return FAILURE;
                }
                stream<<"\t\t</body>"<<endl;
            }
            else
                stream<<"\t\t<filename>"<<bodyVec[i]->getFilename().latin1()<<"</filename>"<<endl;
            stream<<"\t\t<transform>" <<endl;
            stream<< "\t\t\t<fullTransform>"<< bodyVec[i]->getTran() << "</fullTransform>" << endl;
            stream<<"\t\t</transform>" <<endl;
            stream<<"\t</graspableBody>"<<endl;
        }
    }

    for (i=0;i<numRobots;i++) {
        stream<<"\t<robot>"<<endl;
        stream<<"\t\t<filename>"<<robotVec[i]->getFilename().latin1()<<"</filename>"<<endl;
        stream<<"\t\t<dofValues>";
        robotVec[i]->writeDOFVals(stream);
        stream << "</dofValues>" << endl;
        stream<<"\t\t<transform>" <<endl;
        stream<< "\t\t\t<fullTransform>"<< robotVec[i]->getTran() << "</fullTransform>" << endl;
        stream<<"\t\t</transform>" <<endl;
        stream<<"\t</robot>"<<endl;
    }

    for(i=0;i<numRobots;i++) {
        for (j=0;j<robotVec[i]->getNumChains();j++) {
            KinematicChain *chain = robotVec[i]->getChain(j);
            for (k=0;k<chain->getNumAttachedRobots();k++) {
                stream<<"\t<connection>"<<endl;
                stream<< "\t\t<parentRobot>" << i << "</parentRobot>" << endl;
                stream<< "\t\t<parentChain>" << j << "</parentChain>" << endl;
                for (l=0;l<numRobots;l++)
                    if (chain->getAttachedRobot(k) == robotVec[l]) break;
                stream<< "\t\t<childRobot>" << l << "</childRobot>" << endl;
                if (chain->getAttachedRobot(k)->getMountPiece()) {
                    stream<< "\t\t<mountFilename>" << chain->getAttachedRobot(k)->getMountPiece()->getFilename() << "</mountFilename>" << endl;
                }
                stream<<"\t\t<transform>" <<endl;
                stream<< "\t\t\t<fullTransform>"<< chain->getAttachedRobotOffset(k) << "</fullTransform>" << endl;
                stream<<"\t\t</transform>" <<endl;
                stream<<"\t</connection>"<<endl;
            }
        }
    }
    stream<<"\t<camera>"<<endl;
    float px, py, pz, q1, q2, q3, q4, fd;
    if (myIVmgr) {
        myIVmgr->getCamera(px, py, pz, q1, q2, q3, q4, fd);
        stream<<"\t\t<position>"<<px<<" "<<py<<" "<<pz<<"</position>"<<endl;
        stream<<"\t\t<orientation>"<<q1<<" "<<q2<<" "<<q3<<" "<<q4<<"</orientation>"<<endl;
        stream<<"\t\t<focalDistance>"<<fd<<"</focalDistance>"<<endl;
    }
    stream<<"\t</camera>"<<endl;
    stream<<"</world>"<<endl;*/
}

