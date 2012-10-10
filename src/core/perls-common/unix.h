#ifndef __PERLS_COMMON_UNIX_H__
#define __PERLS_COMMON_UNIX_H__

#ifdef __cplusplus
extern "C" {
#endif    

/* Function with behaviour like `mkdir -p'
 * e.g., mode = 0775 
 * returns 0 success; -1 error 
 */
int
unix_mkpath (const char *path, mode_t mode);


/* checks for existence of a running process id
 * returns 0 if pid is alive; -1 if not found
 */
int
unix_pidstat (pid_t pid);


#ifdef __cplusplus
}
#endif

#endif // __PERLS_COMMON_UNIX_H__
