"""
package_name: filtering

Overview:
filtering is a concept we apply in our viam-ros integration to only collect data from topics when an
"event" is occurring (plus and minus event times)

When an event occurs, this event will be added to the table as well as the time the event started. This
will ensure each component which is configured for filtering will only capture cloud aware data when one
or more active events are occurring
"""