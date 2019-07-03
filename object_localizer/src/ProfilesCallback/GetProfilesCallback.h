#ifndef GETPROFILESCALLBACK_H
#define GETPROFILESCALLBACK_H

#include <iostream>
#include <llt.h>
#include <vector>

#define MAX_INTERFACE_COUNT 5
#define MAX_RESOLUTION 6

gint32 GetProfilesCallback(void);
void DisplayProfiles(double *, double *, guint32);
void DisplayTimestamp(guchar *);

void NewProfile(const void *data, size_t data_size, gpointer user_data);
void ControlLostCallback(gpointer user_data);

CInterfaceLLT *hLLT;
guint32 resolution;
std::vector<guint8> profile_buffer;
TScannerType llt_type;
guint32 profile_count, needed_profile_count;
guint32 profile_data_size;

// for live profile count
guint32 first_trans = 0;
guint32 profile_counter = 0, old_profile_counter = 0, lost_profiles = 0;
double shutter_closed = 0, shutter_opened = 0;

// event handle
EHANDLE *event;

#endif // GETPROFILESCALLBACK_H
