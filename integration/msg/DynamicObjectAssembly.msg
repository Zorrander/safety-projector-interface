##A requests to set a specific assembly step  to an existing specific dynamic object instance.

##The temporary unique id of the dynamic object instance, to allow reference to multiple objects of the same type.
string instance_id

##The assembly step of the object to use. For this interface the dynamic objects that do not have an assembly hierarchy are also modelled as assemblies with a single assembly step.
int32 assembly_step
