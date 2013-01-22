    // TODO remove, should not be needed.
    bool CMC7PlainSearchProcessor::findKeywords(const std::string& source, const std::vector<std::string>& keywords) {
        size_t index;
        for (std::vector<std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it) {
            index = source.rfind(*it);
            if (index != std::string::npos) {
                return 1;
            }
        }
        return 0;
    }

