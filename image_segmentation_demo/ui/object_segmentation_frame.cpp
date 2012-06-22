/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Dec 21 2009)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "object_segmentation_frame.h"

///////////////////////////////////////////////////////////////////////////

ObjectSegmentationFrame::ObjectSegmentationFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style, const wxString& name ) : wxFrame( parent, id, title, pos, size, style, name )
{
	this->SetSizeHints( wxDefaultSize, wxSize( -1,-1 ) );
	this->SetBackgroundColour( wxColour( 135, 136, 138 ) );

	wxInitAllImageHandlers();	
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	MainPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );
	MainPanel->SetBackgroundColour( wxColour( 255, 255, 255 ) );
	
	wxBoxSizer* MainSizer;
	MainSizer = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* HeaderSizer;
	HeaderSizer = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* LogoSizer;
	LogoSizer = new wxBoxSizer( wxHORIZONTAL );
	
	
	LogoSizer->Add( 10, 0, 0, 0, 5 );
	
	TitleText = new wxStaticText( MainPanel, wxID_ANY, wxT("Bosch Shared Autonomy Demonstration"), wxDefaultPosition, wxDefaultSize, 0 );
	TitleText->Wrap( -1 );
	TitleText->SetFont( wxFont( 22, 70, 90, 90, false, wxT("Bosch Sans") ) );
	TitleText->SetForegroundColour( wxColour( 64, 66, 69 ) );
	TitleText->SetBackgroundColour( wxColour( 255, 255, 255 ) );
	
	LogoSizer->Add( TitleText, 0, wxTOP, 15 );
	
	
	LogoSizer->Add( 0, 0, 5, wxEXPAND, 5 );
	
	BoschLogo = new wxStaticBitmap( MainPanel, wxID_ANY, wxBitmap( wxT("ui/Bosch_Logo_small.png"), wxBITMAP_TYPE_PNG ), wxDefaultPosition, wxSize( -1,-1 ), 0 );
	BoschLogo->SetBackgroundColour( wxColour( 255, 255, 255 ) );
	
	LogoSizer->Add( BoschLogo, 0, wxTOP|wxBOTTOM|wxRIGHT, 10 );
	
	HeaderSizer->Add( LogoSizer, 0, wxEXPAND, 1 );
	
	NavigationPanel = new wxPanel( MainPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );
	NavigationPanel->SetBackgroundColour( wxColour( 0, 59, 106 ) );
	NavigationPanel->SetMinSize( wxSize( -1,30 ) );
	NavigationPanel->SetMaxSize( wxSize( -1,30 ) );
	
	HeaderSizer->Add( NavigationPanel, 0, wxRIGHT|wxLEFT|wxEXPAND, 10 );
	
	MainSizer->Add( HeaderSizer, 0, wxEXPAND, 10 );
	
	ImagePanel = new wxPanel( MainPanel, wxID_ANY, wxDefaultPosition, wxSize( 640,480 ), 0 );
	ImagePanel->SetBackgroundColour( wxColour( 0, 20, 42 ) );
	ImagePanel->SetMinSize( wxSize( 640,480 ) );
	
	MainSizer->Add( ImagePanel, 10, wxALIGN_CENTER_HORIZONTAL|wxALL|wxEXPAND|wxSHAPED, 10 );
	
	InstructionsLabel = new wxStaticText( MainPanel, wxID_ANY, wxT("Draw a box around an object and push Select. Press Go to pick up the object."), wxDefaultPosition, wxDefaultSize, 0 );
	InstructionsLabel->Wrap( -1 );
	InstructionsLabel->SetFont( wxFont( 14, 70, 90, 90, false, wxT("Bosch Sans") ) );
	
	MainSizer->Add( InstructionsLabel, 0, wxBOTTOM|wxRIGHT|wxLEFT|wxEXPAND, 10 );
	
	wxBoxSizer* ButtonSizer;
	ButtonSizer = new wxBoxSizer( wxHORIZONTAL );
	
	
	ButtonSizer->Add( 0, 0, 3, wxEXPAND, 5 );
	
	wxBoxSizer* SegmentSizer;
	SegmentSizer = new wxBoxSizer( wxVERTICAL );
	
	segment_button_ = new wxBitmapButton( MainPanel, wxID_ANY, wxBitmap( wxT("ui/Arcade_Blue_Up.png"), wxBITMAP_TYPE_PNG ), wxDefaultPosition, wxDefaultSize, 0|wxNO_BORDER );
	
	segment_button_->SetBitmapSelected( wxBitmap( wxT("ui/Arcade_Blue_Down.png"), wxBITMAP_TYPE_PNG ) );
	segment_button_->SetForegroundColour( wxSystemSettings::GetColour( wxSYS_COLOUR_APPWORKSPACE ) );
	segment_button_->SetBackgroundColour( wxSystemSettings::GetColour( wxSYS_COLOUR_APPWORKSPACE ) );
	
	segment_button_->SetForegroundColour( wxSystemSettings::GetColour( wxSYS_COLOUR_APPWORKSPACE ) );
	segment_button_->SetBackgroundColour( wxSystemSettings::GetColour( wxSYS_COLOUR_APPWORKSPACE ) );
	
	SegmentSizer->Add( segment_button_, 0, wxALIGN_CENTER, 0 );
	
	SegmentText = new wxStaticText( MainPanel, wxID_ANY, wxT("Select"), wxDefaultPosition, wxDefaultSize, 0 );
	SegmentText->Wrap( -1 );
	SegmentText->SetFont( wxFont( 14, 70, 90, 90, false, wxT("Bosch Sans") ) );
	
	SegmentSizer->Add( SegmentText, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	ButtonSizer->Add( SegmentSizer, 0, wxEXPAND, 5 );
	
	
	ButtonSizer->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxBoxSizer* AcceptSizer;
	AcceptSizer = new wxBoxSizer( wxVERTICAL );
	
	accept_button_ = new wxBitmapButton( MainPanel, wxID_ANY, wxBitmap( wxT("ui/Arcade_Green_Up.png"), wxBITMAP_TYPE_PNG ), wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW|wxNO_BORDER );
	
	accept_button_->SetBitmapSelected( wxBitmap( wxT("ui/Arcade_Green_Down.png"), wxBITMAP_TYPE_PNG ) );
	AcceptSizer->Add( accept_button_, 0, wxALIGN_CENTER, 10 );
	
	AcceptText = new wxStaticText( MainPanel, wxID_ANY, wxT("Go!"), wxDefaultPosition, wxDefaultSize, 0 );
	AcceptText->Wrap( -1 );
	AcceptText->SetFont( wxFont( 14, 70, 90, 90, false, wxT("Bosch Sans") ) );
	
	AcceptSizer->Add( AcceptText, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	ButtonSizer->Add( AcceptSizer, 0, wxEXPAND, 5 );
	
	
	ButtonSizer->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxBoxSizer* ResetSizer;
	ResetSizer = new wxBoxSizer( wxVERTICAL );
	
	reset_button_ = new wxBitmapButton( MainPanel, wxID_ANY, wxBitmap( wxT("ui/Arcade_Red_Up.png"), wxBITMAP_TYPE_PNG ), wxDefaultPosition, wxDefaultSize, 0|wxNO_BORDER );
	
	reset_button_->SetBitmapSelected( wxBitmap( wxT("ui/Arcade_Red_Down.png"), wxBITMAP_TYPE_PNG ) );
	ResetSizer->Add( reset_button_, 0, wxALIGN_CENTER, 5 );
	
	ResetText = new wxStaticText( MainPanel, wxID_ANY, wxT("Reset"), wxDefaultPosition, wxDefaultSize, 0 );
	ResetText->Wrap( -1 );
	ResetText->SetFont( wxFont( 14, 70, 90, 90, false, wxT("Bosch Sans") ) );
	
	ResetSizer->Add( ResetText, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	ButtonSizer->Add( ResetSizer, 0, wxEXPAND, 5 );
	
	
	ButtonSizer->Add( 0, 0, 3, wxEXPAND, 5 );
	
	MainSizer->Add( ButtonSizer, 0, wxEXPAND, 0 );
	
	FauxFooter = new wxPanel( MainPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	FauxFooter->SetBackgroundColour( wxColour( 255, 255, 255 ) );
	FauxFooter->SetMinSize( wxSize( -1,30 ) );
	FauxFooter->SetMaxSize( wxSize( -1,30 ) );
	
	MainSizer->Add( FauxFooter, 0, wxEXPAND, 0 );
	
	MainPanel->SetSizer( MainSizer );
	MainPanel->Layout();
	MainSizer->Fit( MainPanel );
	bSizer1->Add( MainPanel, 1, wxEXPAND|wxBOTTOM|wxRIGHT|wxLEFT|wxALIGN_CENTER_HORIZONTAL, 10 );
	
	Footer = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	Footer->SetBackgroundColour( wxColour( 135, 136, 138 ) );
	
	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer( wxHORIZONTAL );
	
	refresh_button_ = new wxButton( Footer, wxID_ANY, wxT("Refresh"), wxDefaultPosition, wxDefaultSize, 0 );
	refresh_button_->SetForegroundColour( wxColour( 255, 255, 255 ) );
	refresh_button_->SetBackgroundColour( wxColour( 135, 136, 138 ) );
	
	bSizer11->Add( refresh_button_, 0, wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText4 = new wxStaticText( Footer, wxID_ANY, wxT("Copyright 2012 Robert Bosch LLC. All rights reserved."), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText4->Wrap( -1 );
	m_staticText4->SetForegroundColour( wxColour( 255, 255, 255 ) );
	
	bSizer11->Add( m_staticText4, 0, wxALL, 5 );
	
	Footer->SetSizer( bSizer11 );
	Footer->Layout();
	bSizer11->Fit( Footer );
	bSizer1->Add( Footer, 0, wxEXPAND|wxALL, 10 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	// Connect Events
	segment_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::segmentButtonClicked ), NULL, this );
	accept_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::acceptButtonClicked ), NULL, this );
	reset_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::resetButtonClicked ), NULL, this );
	refresh_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::refreshButtonClicked ), NULL, this );
}

ObjectSegmentationFrame::~ObjectSegmentationFrame()
{
	// Disconnect Events
	segment_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::segmentButtonClicked ), NULL, this );
	accept_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::acceptButtonClicked ), NULL, this );
	reset_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::resetButtonClicked ), NULL, this );
	refresh_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::refreshButtonClicked ), NULL, this );
}
