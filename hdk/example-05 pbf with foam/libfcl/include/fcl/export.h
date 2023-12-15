
#ifndef FCL_EXPORT_H
#define FCL_EXPORT_H

#ifdef FCL_STATIC_DEFINE
#  define FCL_EXPORT
#  define FCL_NO_EXPORT
#else
#  ifndef FCL_EXPORT
#    ifdef fcl_EXPORTS
        /* We are building this library */
#      define FCL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define FCL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef FCL_NO_EXPORT
#    define FCL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef FCL_DEPRECATED
#  define FCL_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef FCL_DEPRECATED_EXPORT
#  define FCL_DEPRECATED_EXPORT FCL_EXPORT FCL_DEPRECATED
#endif

#ifndef FCL_DEPRECATED_NO_EXPORT
#  define FCL_DEPRECATED_NO_EXPORT FCL_NO_EXPORT FCL_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef FCL_NO_DEPRECATED
#    define FCL_NO_DEPRECATED
#  endif
#endif

#endif /* FCL_EXPORT_H */
