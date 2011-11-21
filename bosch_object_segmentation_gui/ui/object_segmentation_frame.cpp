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
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxVERTICAL );
	
	m_panel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 640,480 ), wxTAB_TRAVERSAL );
	bSizer3->Add( m_panel, 0, wxEXPAND | wxALL, 5 );
	
	bSizer1->Add( bSizer3, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer41;
	bSizer41 = new wxBoxSizer( wxHORIZONTAL );
	
	bottom_label_ = new wxStaticText( this, wxID_ANY, wxT("MyLabel"), wxDefaultPosition, wxDefaultSize, 0 );
	bottom_label_->Wrap( -1 );
	bSizer41->Add( bottom_label_, 0, wxALL, 5 );
	
	bSizer1->Add( bSizer41, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxHORIZONTAL );
	
	accept_button_ = new wxButton( this, wxID_ANY, wxT("OK"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( accept_button_, 0, wxALL, 5 );
	
	cancel_button_ = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( cancel_button_, 0, wxALL, 5 );
	
	
	bSizer4->Add( 0, 0, 1, wxEXPAND, 5 );
	
	segment_button_ = new wxButton( this, wxID_ANY, wxT("Segment"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( segment_button_, 0, wxALL, 5 );
	
	reset_button_ = new wxButton( this, wxID_ANY, wxT("Reset"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( reset_button_, 0, wxALL, 5 );
	
	bSizer1->Add( bSizer4, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	// Connect Events
	accept_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::acceptButtonClicked ), NULL, this );
	cancel_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::cancelButtonClicked ), NULL, this );
	segment_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::segmentButtonClicked ), NULL, this );
	reset_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::resetButtonClicked ), NULL, this );
}

ObjectSegmentationFrame::~ObjectSegmentationFrame()
{
	// Disconnect Events
	accept_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::acceptButtonClicked ), NULL, this );
	cancel_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::cancelButtonClicked ), NULL, this );
	segment_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::segmentButtonClicked ), NULL, this );
	reset_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ObjectSegmentationFrame::resetButtonClicked ), NULL, this );
}
