/*
 * Copyright (C) 2003, 2006, 2007, 2008, 2009 Apple Inc. All rights reserved.
 *           (C) 2006 Graham Dennis (graham.dennis@gmail.com)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE COMPUTER, INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE COMPUTER, INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */

#ifndef Settings_h
#define Settings_h

#include "AtomicString.h"
#include "FontRenderingMode.h"
#include "KURL.h"

namespace WebCore {

    class Page;

    enum EditableLinkBehavior {
        EditableLinkDefaultBehavior,
        EditableLinkAlwaysLive,
        EditableLinkOnlyLiveWithShiftKey,
        EditableLinkLiveWhenNotFocused,
        EditableLinkNeverLive
    };

    enum TextDirectionSubmenuInclusionBehavior {
        TextDirectionSubmenuNeverIncluded,
        TextDirectionSubmenuAutomaticallyIncluded,
        TextDirectionSubmenuAlwaysIncluded
    };

    // There are multiple editing details that are different on Windows than Macintosh.
    // We use a single switch for all of them. Some examples:
    //
    //    1) Clicking below the last line of an editable area puts the caret at the end
    //       of the last line on Mac, but in the middle of the last line on Windows.
    //    2) Pushing the down arrow key on the last line puts the caret at the end of the
    //       last line on Mac, but does nothing on Windows. A similar case exists on the
    //       top line.
    //
    // This setting is intended to control these sorts of behaviors. There are some other
    // behaviors with individual function calls on EditorClient (smart copy and paste and
    // selecting the space after a double click) that could be combined with this if
    // if possible in the future.
    enum EditingBehavior { EditingMacBehavior, EditingWindowsBehavior };

    class Settings : public Noncopyable {
    public:
        Settings(Page*);

#ifdef ANDROID_LAYOUT
        // FIXME: How do we determine the margins other than guessing?
        #define ANDROID_SSR_MARGIN_PADDING  3
        #define ANDROID_FCTS_MARGIN_PADDING  10

        enum LayoutAlgorithm {
            kLayoutNormal,
            kLayoutSSR,
            kLayoutFitColumnToScreen
        };
#endif
        void setStandardFontFamily(const AtomicString&);
        const AtomicString& standardFontFamily() const { return m_standardFontFamily; }

        void setFixedFontFamily(const AtomicString&);
        const AtomicString& fixedFontFamily() const { return m_fixedFontFamily; }

#ifdef ANDROID_LAYOUT
        LayoutAlgorithm layoutAlgorithm() const { return m_layoutAlgorithm; }
        void setLayoutAlgorithm(LayoutAlgorithm algorithm) { m_layoutAlgorithm = algorithm; }

        bool useWideViewport() const { return m_useWideViewport; }
        void setUseWideViewport(bool use) { m_useWideViewport = use; }
#endif

        void setSerifFontFamily(const AtomicString&);
        const AtomicString& serifFontFamily() const { return m_serifFontFamily; }

        void setSansSerifFontFamily(const AtomicString&);
        const AtomicString& sansSerifFontFamily() const { return m_sansSerifFontFamily; }

        void setCursiveFontFamily(const AtomicString&);
        const AtomicString& cursiveFontFamily() const { return m_cursiveFontFamily; }

        void setFantasyFontFamily(const AtomicString&);
        const AtomicString& fantasyFontFamily() const { return m_fantasyFontFamily; }

        void setMinimumFontSize(int);
        int minimumFontSize() const { return m_minimumFontSize; }

        void setMinimumLogicalFontSize(int);
        int minimumLogicalFontSize() const { return m_minimumLogicalFontSize; }

        void setDefaultFontSize(int);
        int defaultFontSize() const { return m_defaultFontSize; }

        void setDefaultFixedFontSize(int);
        int defaultFixedFontSize() const { return m_defaultFixedFontSize; }

        // Unlike areImagesEnabled, this only suppresses the network load of
        // the image URL.  A cached image will still be rendered if requested.
        void setLoadsImagesAutomatically(bool);
        bool loadsImagesAutomatically() const { return m_loadsImagesAutomatically; }

#ifdef ANDROID_BLOCK_NETWORK_IMAGE
        void setBlockNetworkImage(bool);
        bool blockNetworkImage() const { return m_blockNetworkImage; }
#endif
        void setJavaScriptEnabled(bool);
        bool isJavaScriptEnabled() const { return m_isJavaScriptEnabled; }

        void setWebSecurityEnabled(bool);
        bool isWebSecurityEnabled() const { return m_isWebSecurityEnabled; }

        void setAllowUniversalAccessFromFileURLs(bool);
        bool allowUniversalAccessFromFileURLs() const { return m_allowUniversalAccessFromFileURLs; }

        void setJavaScriptCanOpenWindowsAutomatically(bool);
        bool javaScriptCanOpenWindowsAutomatically() const { return m_javaScriptCanOpenWindowsAutomatically; }

        void setJavaEnabled(bool);
        bool isJavaEnabled() const { return m_isJavaEnabled; }

        void setImagesEnabled(bool);
        bool areImagesEnabled() const { return m_areImagesEnabled; }

        void setPluginsEnabled(bool);
        bool arePluginsEnabled() const { return m_arePluginsEnabled; }

#ifdef ANDROID_PLUGINS
        void setPluginsOnDemand(bool onDemand) { m_pluginsOnDemand = onDemand; }
        bool arePluginsOnDemand() const { return m_pluginsOnDemand; }
#endif

        void setDatabasesEnabled(bool);
        bool databasesEnabled() const { return m_databasesEnabled; }

        void setLocalStorageEnabled(bool);
        bool localStorageEnabled() const { return m_localStorageEnabled; }

        void setLocalStorageQuota(unsigned);
        unsigned localStorageQuota() const { return m_localStorageQuota; }

        void setPrivateBrowsingEnabled(bool);
        bool privateBrowsingEnabled() const { return m_privateBrowsingEnabled; }

        void setCaretBrowsingEnabled(bool);
        bool caretBrowsingEnabled() const { return m_caretBrowsingEnabled; }

        void setDefaultTextEncodingName(const String&);
        const String& defaultTextEncodingName() const { return m_defaultTextEncodingName; }
        
        void setUsesEncodingDetector(bool);
        bool usesEncodingDetector() const { return m_usesEncodingDetector; }

        void setUserStyleSheetLocation(const KURL&);
        const KURL& userStyleSheetLocation() const { return m_userStyleSheetLocation; }

        void setShouldPrintBackgrounds(bool);
        bool shouldPrintBackgrounds() const { return m_shouldPrintBackgrounds; }

        void setTextAreasAreResizable(bool);
        bool textAreasAreResizable() const { return m_textAreasAreResizable; }

        void setEditableLinkBehavior(EditableLinkBehavior);
        EditableLinkBehavior editableLinkBehavior() const { return m_editableLinkBehavior; }

        void setTextDirectionSubmenuInclusionBehavior(TextDirectionSubmenuInclusionBehavior);
        TextDirectionSubmenuInclusionBehavior textDirectionSubmenuInclusionBehavior() const { return m_textDirectionSubmenuInclusionBehavior; }

#if ENABLE(DASHBOARD_SUPPORT)
        void setUsesDashboardBackwardCompatibilityMode(bool);
        bool usesDashboardBackwardCompatibilityMode() const { return m_usesDashboardBackwardCompatibilityMode; }
#endif
        
        void setNeedsAdobeFrameReloadingQuirk(bool);
        bool needsAcrobatFrameReloadingQuirk() const { return m_needsAdobeFrameReloadingQuirk; }

        void setNeedsKeyboardEventDisambiguationQuirks(bool);
        bool needsKeyboardEventDisambiguationQuirks() const { return m_needsKeyboardEventDisambiguationQuirks; }

        void setTreatsAnyTextCSSLinkAsStylesheet(bool);
        bool treatsAnyTextCSSLinkAsStylesheet() const { return m_treatsAnyTextCSSLinkAsStylesheet; }

        void setNeedsLeopardMailQuirks(bool);
        bool needsLeopardMailQuirks() const { return m_needsLeopardMailQuirks; }

        void setNeedsTigerMailQuirks(bool);
        bool needsTigerMailQuirks() const { return m_needsTigerMailQuirks; }

        void setDOMPasteAllowed(bool);
        bool isDOMPasteAllowed() const { return m_isDOMPasteAllowed; }
        
        void setUsesPageCache(bool);
        bool usesPageCache() const { return m_usesPageCache; }

        void setShrinksStandaloneImagesToFit(bool);
        bool shrinksStandaloneImagesToFit() const { return m_shrinksStandaloneImagesToFit; }

        void setShowsURLsInToolTips(bool);
        bool showsURLsInToolTips() const { return m_showsURLsInToolTips; }

        void setFTPDirectoryTemplatePath(const String&);
        const String& ftpDirectoryTemplatePath() const { return m_ftpDirectoryTemplatePath; }
        
        void setForceFTPDirectoryListings(bool);
        bool forceFTPDirectoryListings() const { return m_forceFTPDirectoryListings; }
        
        void setDeveloperExtrasEnabled(bool);
        bool developerExtrasEnabled() const { return m_developerExtrasEnabled; }

        void setFrameSetFlatteningEnabled(bool);
        bool frameSetFlatteningEnabled() const { return m_frameSetFlatteningEnabled; }

#ifdef ANDROID_META_SUPPORT
        void resetMetadataSettings();
        void setMetadataSettings(const String& key, const String& value);

        void setViewportWidth(int);
        int viewportWidth() const { return m_viewport_width; }

        void setViewportHeight(int);
        int viewportHeight() const { return m_viewport_height; }

        void setViewportInitialScale(int);
        int viewportInitialScale() const { return m_viewport_initial_scale; }

        void setViewportMinimumScale(int);
        int viewportMinimumScale() const { return m_viewport_minimum_scale; }

        void setViewportMaximumScale(int);
        int viewportMaximumScale() const { return m_viewport_maximum_scale; }

        void setViewportUserScalable(bool);
        bool viewportUserScalable() const { return m_viewport_user_scalable; }

        void setViewportTargetDensityDpi(int);
        int viewportTargetDensityDpi() const { return m_viewport_target_densitydpi; }

        void setFormatDetectionAddress(bool);
        bool formatDetectionAddress() const { return m_format_detection_address; }

        void setFormatDetectionEmail(bool);
        bool formatDetectionEmail() const { return m_format_detection_email; }

        void setFormatDetectionTelephone(bool);
        bool formatDetectionTelephone() const { return m_format_detection_telephone; }
#endif
#ifdef ANDROID_MULTIPLE_WINDOWS
        bool supportMultipleWindows() const { return m_supportMultipleWindows; }
        void setSupportMultipleWindows(bool support) { m_supportMultipleWindows = support; }
#endif
        void setAuthorAndUserStylesEnabled(bool);
        bool authorAndUserStylesEnabled() const { return m_authorAndUserStylesEnabled; }
        
        void setFontRenderingMode(FontRenderingMode mode);
        FontRenderingMode fontRenderingMode() const;

        void setNeedsSiteSpecificQuirks(bool);
        bool needsSiteSpecificQuirks() const { return m_needsSiteSpecificQuirks; }
        
        void setWebArchiveDebugModeEnabled(bool);
        bool webArchiveDebugModeEnabled() const { return m_webArchiveDebugModeEnabled; }

        void setLocalFileContentSniffingEnabled(bool);
        bool localFileContentSniffingEnabled() const { return m_localFileContentSniffingEnabled; }

        void setLocalStorageDatabasePath(const String&);
        const String& localStorageDatabasePath() const { return m_localStorageDatabasePath; }
        
        void setApplicationChromeMode(bool);
        bool inApplicationChromeMode() const { return m_inApplicationChromeMode; }

        void setOfflineWebApplicationCacheEnabled(bool);
        bool offlineWebApplicationCacheEnabled() const { return m_offlineWebApplicationCacheEnabled; }

        void setShouldPaintCustomScrollbars(bool);
        bool shouldPaintCustomScrollbars() const { return m_shouldPaintCustomScrollbars; }

        void setZoomsTextOnly(bool);
        bool zoomsTextOnly() const { return m_zoomsTextOnly; }
        
        void setEnforceCSSMIMETypeInStrictMode(bool);
        bool enforceCSSMIMETypeInStrictMode() { return m_enforceCSSMIMETypeInStrictMode; }

        void setMaximumDecodedImageSize(size_t size) { m_maximumDecodedImageSize = size; }
        size_t maximumDecodedImageSize() const { return m_maximumDecodedImageSize; }

#if USE(SAFARI_THEME)
        // Windows debugging pref (global) for switching between the Aqua look and a native windows look.
        static void setShouldPaintNativeControls(bool);
        static bool shouldPaintNativeControls() { return gShouldPaintNativeControls; }
#endif

        void setAllowScriptsToCloseWindows(bool);
        bool allowScriptsToCloseWindows() const { return m_allowScriptsToCloseWindows; }

        void setEditingBehavior(EditingBehavior behavior) { m_editingBehavior = behavior; }
        EditingBehavior editingBehavior() const { return static_cast<EditingBehavior>(m_editingBehavior); }
        
        void setDownloadableBinaryFontsEnabled(bool);
        bool downloadableBinaryFontsEnabled() const { return m_downloadableBinaryFontsEnabled; }

        void setXSSAuditorEnabled(bool);
        bool xssAuditorEnabled() const { return m_xssAuditorEnabled; }

        void setAcceleratedCompositingEnabled(bool);
        bool acceleratedCompositingEnabled() const { return m_acceleratedCompositingEnabled; }

        void setShowDebugBorders(bool);
        bool showDebugBorders() const { return m_showDebugBorders; }

        void setShowRepaintCounter(bool);
        bool showRepaintCounter() const { return m_showRepaintCounter; }

        void setExperimentalNotificationsEnabled(bool);
        bool experimentalNotificationsEnabled() const { return m_experimentalNotificationsEnabled; }

#if PLATFORM(WIN) || (OS(WINDOWS) && PLATFORM(WX))
        static void setShouldUseHighResolutionTimers(bool);
        static bool shouldUseHighResolutionTimers() { return gShouldUseHighResolutionTimers; }
#endif

        void setPluginAllowedRunTime(unsigned);
        unsigned pluginAllowedRunTime() const { return m_pluginAllowedRunTime; }

        void setWebGLEnabled(bool);
        bool webGLEnabled() const { return m_webGLEnabled; }

        void setGeolocationEnabled(bool);
        bool geolocationEnabled() const { return m_geolocationEnabled; }

        void setLoadDeferringEnabled(bool);
        bool loadDeferringEnabled() const { return m_loadDeferringEnabled; }
	//SAMSUNG CHANGE BEGIN 
	void setAdvancedSelectionEnabled(bool);
	bool advancedSelectionEnabled() const { return m_advancedSelectionEnabled; }
	//SAMSUNG CHANGE END

	//SISO_HTMLCOMPOSER begin 
	void setEditableSupportEnabled(bool);
	bool editableSupportEnabled() const { return m_editableSupportEnabled; }
	//SISO_HTMLCOMPOSER end
	

    private:
        Page* m_page;
        
        String m_defaultTextEncodingName;
        String m_ftpDirectoryTemplatePath;
        String m_localStorageDatabasePath;
        KURL m_userStyleSheetLocation;
        AtomicString m_standardFontFamily;
        AtomicString m_fixedFontFamily;
        AtomicString m_serifFontFamily;
        AtomicString m_sansSerifFontFamily;
        AtomicString m_cursiveFontFamily;
        AtomicString m_fantasyFontFamily;
#ifdef ANDROID_LAYOUT
        LayoutAlgorithm m_layoutAlgorithm;
#endif
        EditableLinkBehavior m_editableLinkBehavior;
        TextDirectionSubmenuInclusionBehavior m_textDirectionSubmenuInclusionBehavior;
        int m_minimumFontSize;
        int m_minimumLogicalFontSize;
        int m_defaultFontSize;
        int m_defaultFixedFontSize;
#ifdef ANDROID_META_SUPPORT
        // range is from 200 to 10,000. 0 is a special value means device-width.
        // default is -1, which means undefined.
        int m_viewport_width;
        // range is from 223 to 10,000. 0 is a special value means device-height
        // default is -1, which means undefined.
        int m_viewport_height;
        // range is from 1 to 1000 in percent. default is 0, which means undefined.
        int m_viewport_initial_scale;
        // range is from 1 to 1000 in percent. default is 0, which means undefined.
        int m_viewport_minimum_scale;
        // range is from 1 to 1000 in percent. default is 0, which means undefined.
        int m_viewport_maximum_scale;
        // default is yes
        bool m_viewport_user_scalable : 1;
        // range is from 70 to 400. 0 is a special value means device-dpi
        // default is -1, which means undefined.
        int m_viewport_target_densitydpi;
        // default is yes
        bool m_format_detection_telephone : 1;
        // default is yes
        bool m_format_detection_address : 1;
        // default is yes
        bool m_format_detection_email : 1;
#endif
#ifdef ANDROID_LAYOUT
        bool m_useWideViewport : 1;
#endif
#ifdef ANDROID_MULTIPLE_WINDOWS
        bool m_supportMultipleWindows : 1;
#endif
#ifdef ANDROID_BLOCK_NETWORK_IMAGE
        bool m_blockNetworkImage : 1;
#endif
        size_t m_maximumDecodedImageSize;
        unsigned m_localStorageQuota;
        unsigned m_pluginAllowedRunTime;
        bool m_isJavaEnabled : 1;
        bool m_loadsImagesAutomatically : 1;
        bool m_privateBrowsingEnabled : 1;
        bool m_caretBrowsingEnabled : 1;
        bool m_areImagesEnabled : 1;
        bool m_arePluginsEnabled : 1;
        bool m_databasesEnabled : 1;
        bool m_localStorageEnabled : 1;
        bool m_isJavaScriptEnabled : 1;
        bool m_isWebSecurityEnabled : 1;
        bool m_allowUniversalAccessFromFileURLs: 1;
        bool m_javaScriptCanOpenWindowsAutomatically : 1;
        bool m_shouldPrintBackgrounds : 1;
        bool m_textAreasAreResizable : 1;
#if ENABLE(DASHBOARD_SUPPORT)
        bool m_usesDashboardBackwardCompatibilityMode : 1;
#endif
        bool m_needsAdobeFrameReloadingQuirk : 1;
        bool m_needsKeyboardEventDisambiguationQuirks : 1;
        bool m_treatsAnyTextCSSLinkAsStylesheet : 1;
        bool m_needsLeopardMailQuirks : 1;
        bool m_needsTigerMailQuirks : 1;
        bool m_isDOMPasteAllowed : 1;
        bool m_shrinksStandaloneImagesToFit : 1;
        bool m_usesPageCache: 1;
        bool m_showsURLsInToolTips : 1;
        bool m_forceFTPDirectoryListings : 1;
        bool m_developerExtrasEnabled : 1;
        bool m_authorAndUserStylesEnabled : 1;
        bool m_needsSiteSpecificQuirks : 1;
        unsigned m_fontRenderingMode : 1;
        bool m_frameSetFlatteningEnabled : 1;
        bool m_webArchiveDebugModeEnabled : 1;
        bool m_localFileContentSniffingEnabled : 1;
        bool m_inApplicationChromeMode : 1;
        bool m_offlineWebApplicationCacheEnabled : 1;
        bool m_shouldPaintCustomScrollbars : 1;
        bool m_zoomsTextOnly : 1;
        bool m_enforceCSSMIMETypeInStrictMode : 1;
        bool m_usesEncodingDetector : 1;
        bool m_allowScriptsToCloseWindows : 1;
        unsigned m_editingBehavior : 1;
        bool m_downloadableBinaryFontsEnabled : 1;
        bool m_xssAuditorEnabled : 1;
        bool m_acceleratedCompositingEnabled : 1;
        bool m_showDebugBorders : 1;
        bool m_showRepaintCounter : 1;
        bool m_experimentalNotificationsEnabled : 1;
        bool m_webGLEnabled : 1;
        bool m_geolocationEnabled : 1;
        bool m_loadDeferringEnabled : 1;
#ifdef ANDROID_PLUGINS
        bool m_pluginsOnDemand : 1;
#endif
	//SAMSUNG CHANGE  
	bool m_advancedSelectionEnabled : 1;

	//SISO_HTMLCOMPOSER begin  
	bool m_editableSupportEnabled : 1;
	//SISO_HTMLCOMPOSER end 	

#if USE(SAFARI_THEME)
        static bool gShouldPaintNativeControls;
#endif
#if PLATFORM(WIN) || (OS(WINDOWS) && PLATFORM(WX))
        static bool gShouldUseHighResolutionTimers;
#endif
    };

} // namespace WebCore

#endif // Settings_h
