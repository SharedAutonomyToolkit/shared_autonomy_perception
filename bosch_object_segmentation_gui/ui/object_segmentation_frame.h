///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Dec 21 2009)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __object_segmentation_frame__
#define __object_segmentation_frame__

#include <wx/panel.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/button.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class ObjectSegmentationFrame
///////////////////////////////////////////////////////////////////////////////
class ObjectSegmentationFrame : public wxFrame 
{
	private:
	
	protected:
		wxPanel* m_panel;
		wxStaticText* bottom_label_;
		wxButton* accept_button_;
		wxButton* cancel_button_;
		
		wxButton* segment_button_;
		wxButton* reset_button_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void acceptButtonClicked( wxCommandEvent& event ) { event.Skip(); }
		virtual void cancelButtonClicked( wxCommandEvent& event ) { event.Skip(); }
		virtual void segmentButtonClicked( wxCommandEvent& event ) { event.Skip(); }
		virtual void resetButtonClicked( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		ObjectSegmentationFrame( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Object Segmentation"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 700,700 ), long style = wxCAPTION|wxFRAME_FLOAT_ON_PARENT|wxMINIMIZE_BOX|wxTAB_TRAVERSAL, const wxString& name = wxT("ObjectSegmentationWindow") );
		~ObjectSegmentationFrame();
	
};

#endif //__object_segmentation_frame__
