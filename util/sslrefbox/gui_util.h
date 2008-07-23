/*
 * TITLE: gui_util.h
 *
 * PURPOSE: provide some utilities for gui control
 *
 * WRITTEN BY: Brett Browning 
 * 
 */

/* LICENSE:  =========================================================================
    RoboCup F180 Referee Box Source Code Release
  -------------------------------------------------------------------------
    Copyright (C) 2003 RoboCup Federation
  -------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ------------------------------------------------------------------------- 

 */
 
#ifndef __GUI_UTIL_H__
#define __GUI_UTIL_H__

#include <gtk/gtk.h>

static GtkWidget *NewPackButton(GtkWidget *box, const char *label, int pad, int exp,
                         GtkSignalFunc fp, gpointer data);
static GtkWidget *NewPackCBox(GtkWidget *box, const char *label, int pad, int exp,
                       GtkSignalFunc fp, gpointer data);
static GtkWidget *NewHSeparator(GtkWidget *box);
static GtkWidget *NewRadioButton(GtkWidget *box, GtkWidget *group, const char *label, 
                          int pad, int exp, GtkSignalFunc fp, gpointer data);
static GtkWidget *NewTableRadioButton(GtkWidget *table, GtkWidget *group, 
                               const char *label, 
                               int lc, int rc, int tc, int bc, 
                               GtkSignalFunc fp, gpointer data);
static GtkWidget *NewTableLabel(GtkWidget *table, const char *label, 
                         int lc, int rc, int tc, int bc);
static GtkWidget *CreateLabel(const char *fmt, ...);
static GtkWidget *NewTextBox(GtkWidget *vbox_main, const char *label, 
                      bool editable, char *def_text);
static void SetText(GtkWidget *w, char *buff);


/*
 * NewPackButton -
 * 
 * This function creates a new button with the given label
 * packs into the given box and shows it. 
 *
 * RETURN VALUE: a pointer to The button widget
 */
static GtkWidget *NewPackButton(GtkWidget *box, const char *label, int pad, int exp,
                                GtkSignalFunc fp, gpointer data = NULL)
{
	GtkWidget *button;

	if (label)
		button = gtk_button_new_with_label(label);
	else
		button = gtk_button_new();
	gtk_box_pack_start(GTK_BOX(box), button, exp, exp, pad);
	gtk_widget_show(button);

	/* connect up the signal if there is one */
	if (fp)
		gtk_signal_connect(GTK_OBJECT(button), "clicked", fp, data);
  	
	/* all done */
	return (button);
}


/*
 * NewPackCBox -
 * 
 * This function creates a new button with the given label
 * packs into the given box and shows it. 
 *
 * RETURN VALUE: a pointer to The button widget
 */
static GtkWidget *NewPackCBox(GtkWidget *box, const char *label, int pad, int exp,
                              GtkSignalFunc fp, gpointer data = NULL)
{
	GtkWidget *button;

	if (label)
		button = gtk_check_button_new_with_label(label);
	else
		button = gtk_check_button_new();
	gtk_box_pack_start(GTK_BOX(box), button, exp, exp, pad);
	gtk_widget_show(button);

	/* connect up the signal if there is one */
	if (fp)
		gtk_signal_connect(GTK_OBJECT(button), "clicked", fp, data);
  	
	/* all done */
	return (button);
}


/*
 * NewHSeparator -
 *
 * create a H Separator and add it to the box 
 */
static GtkWidget *NewHSeparator(GtkWidget *box)
{
	GtkWidget *w;
	
	w = gtk_hseparator_new();
	gtk_box_pack_start(GTK_BOX(box), w, true, true, 0);
	gtk_widget_show(w);
	return (w);
}


/*
 * NewRadioButton -
 *
 * Create a new radio button maybe with a label and attach
 * it to the given group and place it in the given box
 */
static GtkWidget *NewRadioButton(GtkWidget *box, GtkWidget *group, const char *label, 
                                 int pad, int exp, GtkSignalFunc fp, gpointer data)
{
	GtkWidget *w;
	
	if (group != NULL) {
		if (label)
			w = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(group), label);
		else
			w = gtk_radio_button_new_from_widget(GTK_RADIO_BUTTON(group));
	} else {
		if (label)
			w = gtk_radio_button_new_with_label(NULL, label);
		else
			w = gtk_radio_button_new(NULL);
	}
	
	gtk_box_pack_start(GTK_BOX(box), w, exp, exp, pad);
	gtk_widget_show(w);
	
	if (fp)
		gtk_signal_connect(GTK_OBJECT(w), "clicked", fp, data);
		
	return (w);
}


/*
 * NewTableRadioButton -
 *
 * Create a new radio button maybe with a label and attach
 * it to the given group and place it in the given table
 */
static GtkWidget *NewTableRadioButton(GtkWidget *table, GtkWidget *group, 
                                      const char *label, 
                                      int lc, int rc, int tc, int bc, 
                                      GtkSignalFunc fp, gpointer data)
{
	GtkWidget *w;
	
	if (group != NULL) {
		if (label)
			w = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(group), label);
		else
			w = gtk_radio_button_new_from_widget(GTK_RADIO_BUTTON(group));
	} else {
		if (label)
			w = gtk_radio_button_new_with_label(NULL, label);
		else
			w = gtk_radio_button_new(NULL);
	}
	
	gtk_table_attach_defaults(GTK_TABLE(table), w, lc, rc, tc, bc);
	gtk_widget_show(w);
	
	if (fp)
		gtk_signal_connect(GTK_OBJECT(w), "clicked", fp, data);
		
	return (w);
}

/*
 * NewTableLabel -
 *
 * create a label and put it in a table
 */
static GtkWidget *NewTableLabel(GtkWidget *table, const char *label, 
                                int lc, int rc, int tc, int bc)
{
	GtkWidget *w;
	
	/* create the title labels */
	w = CreateLabel(label);
	gtk_table_attach_defaults(GTK_TABLE(table), w, lc, rc, tc, bc);
	gtk_widget_show(w);
	return (w);
}


/*
 * CreateLabe -
 *
 * Creates a label from a format string ala printf
 */
static GtkWidget *CreateLabel(const char *fmt, ...)
{
  char buffer[256];
  GtkWidget *w;
  va_list ap;

  va_start(ap, fmt);
  vsprintf(buffer, fmt, ap);
  w = gtk_label_new(buffer);

  return w;
}


/*
 * NewTextBox -
 *
 * This function creates a new text box and adds it to
 * the given vbox. It returns a pointer to the non-editable (setable)
 * text box that it creates with teh default text given
 *
 * RETURN VALUE: Gtkwidget pointer to the text box
 */
static GtkWidget *NewTextBox(GtkWidget *vbox_main, const char *label = "", 
                             bool editable = false, char *def_text = "")
{
	GtkWidget	*wid, *vbox;
	                 
	/* create hbox to contain the label and etnry box */
	vbox = gtk_vbox_new(FALSE, 0);
	
	/* create the label and put it in the entry box */
	if (label) {
		wid = gtk_label_new(label);
		gtk_box_pack_start(GTK_BOX(vbox), wid, FALSE, TRUE, 2);
		gtk_widget_show(wid);
	}
	
	/* create the entry box,make it uneditable and add it */
	wid = gtk_text_new(NULL, NULL);
	gtk_box_pack_start(GTK_BOX(vbox), wid, TRUE, TRUE, 2);
	gtk_text_set_editable(GTK_TEXT(wid), editable);

	/* set word wrapping to false */
	gtk_text_set_word_wrap(GTK_TEXT(wid), FALSE);
	gtk_text_set_line_wrap(GTK_TEXT(wid), FALSE);
	
	/* set the default text */
	if (def_text != NULL)
		SetText(wid, def_text);
	gtk_widget_show(wid);
	
  	/* add the hbox to the vbox and show it */
	gtk_box_pack_start(GTK_BOX(vbox_main), vbox, FALSE, TRUE, 2);
	gtk_widget_show(vbox);
	
	/* return the entry box widget */
	return (wid);
}	


/*
 * SetText -
 *
 8 Sets the new text for a text box 
 */
static void SetText(GtkWidget *w, char *buff)
{
	int		len = strlen(buff);
	gint	position = 0;
	
	gtk_editable_delete_text(GTK_EDITABLE(w), 0, -1);    
	gtk_editable_insert_text(GTK_EDITABLE(w), buff, len, &position);
}



#endif
